#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TwistStamped, Point, PointStamped
from visualization_msgs.msg import Marker

#from tf_transformations import euler_from_quaternion


class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit')

        # Parameters
        self.lookahead_distance = self.declare_parameter(
            'lookahead_distance', 0.1).value
        self.max_linear_vel = self.declare_parameter(
            'max_linear_velocity', 0.3).value
        self.max_angular_vel = self.declare_parameter(
            'max_angular_velocity', 0.5).value
        self.goal_tolerance = self.declare_parameter(
            'goal_tolerance', 0.15).value

        # State
        self.current_path = None
        self.current_pose = None
        self.path_received = False
        self.odom_received = False

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/map/centerline_path', self.path_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 20)

        # Publisher
        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10)
        
        # Lookahead point visualization
        self.lookahead_pub = self.create_publisher(
            Marker, '/pure_pursuit/lookahead_point', 10)

        # Timer (20 Hz, matches the 50 ms period in the original)
        self.timer = self.create_timer(0.05, self.control_loop)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def path_callback(self, msg: Path):
        self.current_path = msg
        self.path_received = True

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.odom_received = True

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

    def find_lookahead_point(self):
        """Return (Point, is_goal) or (None, False) if no path."""
        if not self.current_path or not self.current_path.poses:
            return None

        robot = self.current_pose.position

        # Find nearest point on path
        nearest_idx = 0
        nearest_dist = float('inf')

        for i, pose_stamped in enumerate(self.current_path.poses):
            d = self.distance(robot, pose_stamped.pose.position)
            if d < nearest_dist:
                nearest_dist = d
                nearest_idx = i

        # Walk forward from nearest point until we exceed lookahead distance
        for i in range(nearest_idx, len(self.current_path.poses)):
            d = self.distance(robot, self.current_path.poses[i].pose.position)
            if d >= self.lookahead_distance:
                return self.current_path.poses[i].pose.position

        # No point far enough away -> use the final point
        return self.current_path.poses[-1].pose.position
    
    def publish_lookahead_marker(self, point: Point):
        marker = Marker()
        #marker.header.frame_id = ''   # match the frame your path is published in
        marker.header = self.current_path.header   # <-- reuse the path's header

        #marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pure_pursuit'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = point.x
        marker.pose.position.y = point.y
        marker.pose.position.z = point.z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0  # persists until overwritten

        self.lookahead_pub.publish(marker)

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        if not self.path_received or not self.odom_received:
            return

        target = self.find_lookahead_point()
        if target is None:
            return
        
        self.publish_lookahead_marker(target)

        yaw = self.get_yaw()

        dx = target.x - self.current_pose.position.x
        dy = target.y - self.current_pose.position.y

        # Transform target into robot frame
        x_r = math.cos(yaw) * dx + math.sin(yaw) * dy
        y_r = -math.sin(yaw) * dx + math.cos(yaw) * dy

        L = math.hypot(x_r, y_r)

        if L < 0.05:
            return

        curvature = (2.0 * y_r) / (L * L)

        v = self.max_linear_vel
        omega = v * curvature
        omega = max(-self.max_angular_vel, min(self.max_angular_vel, omega))

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v
        cmd.twist.angular.z = omega

        # Stop near the end of the path
        goal_point = self.current_path.poses[-1].pose.position
        goal_dist = self.distance(self.current_pose.position, goal_point)

        if goal_dist < self.goal_tolerance:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

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