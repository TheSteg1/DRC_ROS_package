#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Float32, Bool

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, PointStamped
from visualization_msgs.msg import Marker

#from tf_transformations import euler_from_quaternion


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # # Parameters
        # self.lookahead_distance = self.declare_parameter(
        #     'lookahead_distance', 0.3).value
        self.max_linear_vel = self.declare_parameter(
            'max_linear_velocity', 0.5).value
        self.max_angular_vel = self.declare_parameter(
            'max_angular_velocity', 0.3).value
        self.goal_tolerance = self.declare_parameter(
            'goal_tolerance', 0.1).value
        self.k_h = self.declare_parameter(
            'k_h', 0.02).value
        # State
        self.current_pose = None
        self.path_received = False
        self.odom_received = False
        self.lookahead_point = None

        self._enabled       = False #can set to true for SIM 

        # Subscribers
        # self.path_sub = self.create_subscription(
        #     Path, '/map/centerline_path', self.path_callback, 10)

        self.lookahead_sub = self.create_subscription(
            PoseStamped, '/map/pure_pursuit_point', self.lookahead_callback, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20)

        self._enable_sub = self.create_subscription(
            Bool, 'enable_control',
            self._enable_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        
        # # Lookahead point visualization
        # self.lookahead_pub = self.create_publisher(
        #     Marker, '/pure_pursuit/lookahead_point', 10)

        # Timer (20 Hz, matches the 50 ms period in the original)
        self.timer = self.create_timer(0.1, self.control_loop)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _enable_callback(self, msg: Bool):
        if msg.data != self._enabled:
            self.get_logger().info(
                f"Control {'ENABLED' if msg.data else 'DISABLED'} via enable_control topic"
            )
        self._enabled = msg.data
        if not self._enabled:
            # Immediately publish a stop the moment we're disabled, rather
            # than waiting for the next control loop tick.
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.odom_received = True

    def lookahead_callback(self, msg: PoseStamped):
        self.lookahead_point = msg.pose.position

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def distance(a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def get_yaw(self) -> float:
        # q = self.current_pose.orientation
        # _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        # return yaw
        q = self.current_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # def find_lookahead_point(self):
    #     """Return (Point, is_goal) or (None, False) if no path."""
    #     if not self.current_path or not self.current_path.poses:
    #         return None

    #     robot = self.current_pose.position

    #     # Find nearest point on path
    #     best_point = None
    #     best_dist = float('inf')

    #     for pose_stamped in self.current_path.poses:
    #         p = pose_stamped.pose.position
    #         d = self.distance(robot, p)

    #         if d >= self.lookahead_distance and d < best_dist:
    #             best_dist = d
    #             best_point = p

    #     if best_point is not None:
    #         return best_point
        
    #     return self.current_path.poses[-1].pose.position
        
        # # Walk forward from nearest point until we exceed lookahead distance
        # for i in range(nearest_idx, len(self.current_path.poses)):
        #     d = self.distance(robot, self.current_path.poses[i].pose.position)
        #     if d >= self.lookahead_distance:
        #         return self.current_path.poses[i].pose.position

        # # No point far enough away -> use the final point
        # return self.current_path.poses[-1].pose.position
    
    # def publish_lookahead_marker(self, point: Point):
    #     marker = Marker()
    #     #marker.header.frame_id = ''   # match the frame your path is published in
    #     marker.header = self.current_path.header   # <-- reuse the path's header

    #     #marker.header.stamp = self.get_clock().now().to_msg()
    #     marker.ns = 'pure_pursuit'
    #     marker.id = 0
    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD

    #     marker.pose.position.x = point.x
    #     marker.pose.position.y = point.y
    #     marker.pose.position.z = point.z
    #     marker.pose.orientation.w = 1.0

    #     marker.scale.x = 0.15
    #     marker.scale.y = 0.15
    #     marker.scale.z = 0.15

    #     marker.color.r = 1.0
    #     marker.color.g = 0.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0

    #     marker.lifetime.sec = 0  # persists until overwritten

    #     self.lookahead_pub.publish(marker)


    # def control_loop(self):
    #     if not self._enabled:
    #         cmd = TwistStamped()
    #         cmd.header.stamp = self.get_clock().now().to_msg()
    #         cmd.header.frame_id = 'base_link'
    #         cmd.twist.linear.x = 0.0
    #         cmd.twist.angular.z = 0.0
    #         self.cmd_pub.publish(cmd)
    #         return

    #     if not self.odom_received:
    #         return

    #     target = self.lookahead_point
    #     if target is None:
    #         return

    #     yaw = self.get_yaw()

    #     dx = target.x - self.current_pose.position.x
    #     dy = target.y - self.current_pose.position.y

    #     x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
    #     y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy

    #     L = math.hypot(x_r, y_r)

    #     if L < 0.05:
    #         return

    #     curvature = (2.0 * y_r) / (L * L)

    #     v = self.max_linear_vel
    #     omega = v * curvature
    #     omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

    #     cmd = TwistStamped()
    #     cmd.header.stamp = self.get_clock().now().to_msg()
    #     cmd.header.frame_id = 'base_link'
    #     cmd.twist.linear.x = float(v)
    #     cmd.twist.angular.z = float(omega)

    #     # Stop when close to the lookahead point itself
    #     if L < self.goal_tolerance:
    #         cmd.twist.linear.x = 0.0
    #         cmd.twist.angular.z = 0.0
    #     self.get_logger().info('Linear: %.2f Angular: %.2f' % (cmd.twist.linear.x, cmd.twist.angular.z))
    #     self.cmd_pub.publish(cmd)

    def control_loop(self):
        if not self._enabled:
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if not self.odom_received:
            return

        target = self.lookahead_point
        if target is None:
            return

        yaw = self.get_yaw()

        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y

        # Transform target into robot frame
        x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy

        L = math.hypot(x_r, y_r)

        if L < 0.05:
            return

        # Heading error to lookahead point (robot frame)
        theta_star = math.atan2(y_r, x_r)

        
        omega = self.k_h * theta_star
        #v = self.max_linear_vel + (abs(omega) * 0.6)
        v = self.max_linear_vel / (1.0 + abs(theta_star))
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = float(v)
        cmd.twist.angular.z = float(omega)

        # if L < self.goal_tolerance:
        #     cmd.twist.linear.x = 0.0
        #     cmd.twist.angular.z = 0.0
        self.get_logger().info('Linear: %.2f Angular: %.2f' % (cmd.twist.linear.x, cmd.twist.angular.z))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()