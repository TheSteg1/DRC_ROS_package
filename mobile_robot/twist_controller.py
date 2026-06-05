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

Subscriptions:
  colour_error   (std_msgs/Float32)  — lane midpoint offset, [-1, 1]
  obstacle_info  (std_msgs/Float32)  — red obstacle centroid offset, [-1, 1]
                                       0.0 means no obstacle visible

Publications:
  /diff_cont/cmd_vel  (geometry_msgs/TwistStamped)
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
        self._obstacle_size = 0.0
        self._lanes_visible = False
        # Line-follow behaviour
        self.declare_parameter('forward_speed',   0.2)   # m/s cruise speed
        self.declare_parameter('follow_kp',       0.7)   # proportional gain

        # Obstacle-avoid behaviour
        # If obstacle_info != 0.0 AND abs(offset) < avoid_threshold,
        # the obstacle is considered "in the way" and AVOID activates.
        # Raise threshold to trigger avoidance earlier (obstacle further away).
        self.declare_parameter('avoid_threshold',  1.0)  # always avoid if visible
        self.declare_parameter('avoid_speed',      0.1)  # slow down while avoiding
        self.declare_parameter('avoid_kp',         0.9)  # gain for avoidance steer

        # Watchdog: if no message arrives within this many seconds, stop.
        self.declare_parameter('watchdog_timeout', 0.5)

        # --- State ---
        self._lane_error     = 0.0
        self._obstacle_info  = 0.0   # 0.0 = no obstacle
        self._last_error_t   = self.get_clock().now()
        self._last_obs_t     = self.get_clock().now()

        # --- Subscribers ---
        self._error_sub = self.create_subscription(
            Float32,
            'colour_error',
            self._error_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        self._obstacle_sub = self.create_subscription(
            Float32,
            'obstacle_info',
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
        # --- Publisher ---
        self._cmd_pub = self.create_publisher(
            TwistStamped,
            '/diff_cont/cmd_vel',
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        # --- Control loop timer (10 Hz) ---
        # Decoupling the control output from the subscription callbacks means
        # the robot still gets a safe stop command even if messages stop arriving.
        self._timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info('twist_controller started')

    # -----------------------------------------------------------------------
    # Callbacks — just store latest values
    # -----------------------------------------------------------------------

    def _error_callback(self, msg: Float32):
        self._lane_error   = msg.data
        self._last_error_t = self.get_clock().now()

    def _obstacle_callback(self, msg: Float32):
        self._obstacle_info = msg.data
        self._last_obs_t    = self.get_clock().now()

    # -----------------------------------------------------------------------
    # Control loop
    # -----------------------------------------------------------------------

    def _control_loop(self):
        now = self.get_clock().now()
        timeout = self.get_parameter('watchdog_timeout').value

        # Watchdog: if either topic has gone stale, publish zero and bail out
        error_age = (now - self._last_error_t).nanoseconds / 1e9
        obs_age   = (now - self._last_obs_t).nanoseconds   / 1e9

        if error_age > timeout or obs_age > timeout:
            self.get_logger().warn(
                f'Topic stale (colour_error: {error_age:.1f}s, '
                f'obstacle_info: {obs_age:.1f}s) — stopping',
                throttle_duration_sec=1.0,
            )
            self._publish(linear_x=0.0, angular_z=0.0)
            return

        # --- Behaviour arbitration ---
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
        """
        Returns (linear_x, angular_z, active_behaviour_name).

        Suppression hierarchy:
          AVOID fires if obstacle_info != 0.0 (image_processor only publishes
          non-zero when red pixel count exceeds obstacle_min_pixels).
          When AVOID is active it completely suppresses FOLLOW.
        """

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
        """
        Steer away from the obstacle centroid.

        obstacle_info > 0  → obstacle is LEFT  → steer right (negative angular_z)
        obstacle_info < 0  → obstacle is RIGHT → steer left  (positive angular_z)

        We also reduce forward speed so the robot has time to clear the obstacle.
        """
        avoid_kp    = self.get_parameter('avoid_kp').value
        avoid_speed = self.get_parameter('avoid_speed').value
        speed = avoid_speed * max(0.1, 1.0 - self._obstacle_size / 0.20)  # slow down more as obstacle gets bigger (tune 0.20)

        # Steer away: negate the offset so we turn away from the obstacle
        angular_z = avoid_kp * self._obstacle_info

        return speed, angular_z, 'AVOID'

    # -----------------------------------------------------------------------
    # Behaviour 2 — FOLLOW (lower priority)
    # -----------------------------------------------------------------------

    def _follow_behaviour(self) -> tuple[float, float, str]:
        """
        Proportional controller on lane error — identical to the original
        twistController logic.
        """
        forward_speed = self.get_parameter('forward_speed').value
        follow_kp     = self.get_parameter('follow_kp').value

        angular_z = -follow_kp * self._lane_error

        if not self._lanes_visible:
            forward_speed = 0.3  # slow down if we can't see the lanes (tune this)
            angular_z += 0.06 if self._lane_error < 0 else -0.06  # steer in last known error direction (tune this)

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