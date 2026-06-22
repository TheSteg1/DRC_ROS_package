#!/usr/bin/env python3
"""
twist_controller.py
-------------------
Behaviour-based controller with two prioritised behaviours:

  Priority 1 — AVOID:  obstacle detected → slow down, steer away from it
  Priority 2 — FOLLOW: no obstacle       → proportional line-follow steering

Arbitration is suppression-based: when AVOID is active it completely
replaces the FOLLOW output. FOLLOW only drives the robot when AVOID
is inactive.

Start/stop gating:
  Subscribes to 'enable_control' (std_msgs/Bool). The control loop only
  publishes non-zero velocities while enabled == True. Defaults to
  DISABLED on startup as a safety measure — you must explicitly enable it
  (e.g. via keyboard_trigger.py) before the robot moves.

Subscriptions:
  colour_error    (std_msgs/Float32) — lane midpoint offset, [-1, 1]
  obstacle_info   (std_msgs/Float32) — red obstacle centroid offset, [-1, 1]
  obstacle_size   (std_msgs/Float32) — normalised obstacle area, [0, 1]
  lanes_visible   (std_msgs/Bool)    — whether both lane colours are visible
  enable_control  (std_msgs/Bool)    — start/stop gate

Publications:
  /cmd_vel  (geometry_msgs/TwistStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import TwistStamped


class TwistController(Node):

    def __init__(self):
        super().__init__('twist_controller')

        # --- Tunable parameters ---
        self.declare_parameter('avoid_size_threshold', 0.06)  # tune this

        # Line-follow behaviour
        self.declare_parameter('forward_speed', 0.3)   # m/s cruise speed
        self.declare_parameter('follow_kp',     0.5)   # proportional gain

        # Obstacle-avoid behaviour
        self.declare_parameter('avoid_threshold', 1.0)  # always avoid if visible
        self.declare_parameter('avoid_speed',     0.2)  # slow down while avoiding
        self.declare_parameter('avoid_kp',        0.9)  # gain for avoidance steer

        # Watchdog: if no message arrives within this many seconds, stop.
        self.declare_parameter('watchdog_timeout', 0.5)

        # --- State ---
        self._lane_error    = 0.0
        self._obstacle_info = 0.0   # 0.0 = no obstacle
        self._obstacle_size = 0.0
        self._lanes_visible = False
        self._first_time = False
        self._visible_count = 0
        self._last_error_t  = self.get_clock().now()
        self._last_obs_t    = self.get_clock().now()
        self._enabled       = False  # safety default: starts disabled

        # --- Subscribers ---
        self._error_sub = self.create_subscription(
            Float32, 'colour_error',
            self._error_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        self._obstacle_sub = self.create_subscription(
            Float32, 'obstacle_info',
            self._obstacle_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        self._size_sub = self.create_subscription(
            Float32, 'obstacle_size',
            lambda msg: setattr(self, '_obstacle_size', msg.data),
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        self._lanes_visible_sub = self.create_subscription(
            Bool, 'lanes_visible',
            lambda msg: setattr(self, '_lanes_visible', msg.data),
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        self._enable_sub = self.create_subscription(
            Bool, 'enable_control',
            self._enable_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        # --- Control loop timer (50 Hz) ---
        self._timer = self.create_timer(0.02, self._control_loop)

        self.get_logger().info(
            'twist_controller started — DISABLED by default. '
            'Publish True to enable_control to start driving.'
        )

    # -----------------------------------------------------------------------
    # Callbacks — just store latest values
    # -----------------------------------------------------------------------

    def _error_callback(self, msg: Float32):
        self._lane_error   = msg.data
        self._last_error_t = self.get_clock().now()

    def _obstacle_callback(self, msg: Float32):
        self._obstacle_info = msg.data
        self._last_obs_t    = self.get_clock().now()

    def _enable_callback(self, msg: Bool):
        if msg.data != self._enabled:
            self.get_logger().info(
                f"Control {'ENABLED' if msg.data else 'DISABLED'} via enable_control topic"
            )
        self._enabled = msg.data
        if not self._enabled:
            # Immediately publish a stop the moment we're disabled, rather
            # than waiting for the next control loop tick.
            self._publish(0.0, 0.0)

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def _control_loop(self):
        # Gate everything on the enable flag first — this is the highest
        # priority behaviour of all, above even AVOID.
        if not self._enabled:
            self._publish(0.0, 0.0)
            return

        now = self.get_clock().now()
        timeout = self.get_parameter('watchdog_timeout').value

        error_age = (now - self._last_error_t).nanoseconds / 1e9
        obs_age   = (now - self._last_obs_t).nanoseconds   / 1e9

        if error_age > timeout or obs_age > timeout:
            self.get_logger().warn(
                f'Topic stale (colour_error: {error_age:.1f}s, '
                f'obstacle_info: {obs_age:.1f}s) — stopping',
                throttle_duration_sec=1.0,
            )
            self._publish(0.0, 0.0)
            return

        linear_x, angular_z, active = self._arbitrate()

        self.get_logger().info(
            f'[{active:6s}]  lane_err={self._lane_error:+.3f}  '
            f'obs={self._obstacle_info:+.3f}  '
            f'lin={linear_x:.2f}  ang={angular_z:+.3f}',
            throttle_duration_sec=0.5,
        )

        self._publish(linear_x, angular_z)

    # -----------------------------------------------------------------------
    # Behaviour arbitration
    # -----------------------------------------------------------------------

    def _arbitrate(self) -> tuple[float, float, str]:
        if self._obstacle_visible():
            return self._avoid_behaviour()
        else:
            return self._follow_behaviour()

    def _obstacle_visible(self) -> bool:
        threshold = self.get_parameter('avoid_size_threshold').value
        return self._obstacle_info != 0.0 and self._obstacle_size >= threshold

    # -----------------------------------------------------------------------
    # Behaviour 1 — AVOID (higher priority)
    # -----------------------------------------------------------------------

    def _avoid_behaviour(self) -> tuple[float, float, str]:
        avoid_kp    = self.get_parameter('avoid_kp').value
        avoid_speed = self.get_parameter('avoid_speed').value
        speed = avoid_speed * max(0.1, 1.0 - self._obstacle_size / 0.20)

        angular_z = avoid_kp * self._obstacle_info

        return speed, angular_z, 'AVOID'

    # -----------------------------------------------------------------------
    # Behaviour 2 — FOLLOW (lower priority)
    # -----------------------------------------------------------------------

    def _follow_behaviour(self) -> tuple[float, float, str]:
        forward_speed = self.get_parameter('forward_speed').value
        follow_kp     = self.get_parameter('follow_kp').value

        angular_z = -follow_kp * self._lane_error

        if not self._lanes_visible:
            self._first_time = True
            forward_speed = 0.2
            #angular_z += 0.06 if self._lane_error < 0 else -0.06
        if self._lanes_visible and self._first_time:
            self._visible_count += 1 
            angular_z = 0.0
            forward_speed = 0.0
            print("\nthis should be going here\n")
        if self._lanes_visible and self._visible_count >= 10:
            self._visible_count = 0
            self._first_time = False

        return forward_speed, angular_z, 'FOLLOW'

    # -----------------------------------------------------------------------

    def _publish(self, linear_x: float, angular_z: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x  = linear_x
        msg.twist.angular.z = angular_z
        self._cmd_pub.publish(msg)


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TwistController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()