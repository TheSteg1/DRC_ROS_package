import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TwistStamped

import numpy as np


class twistController(Node):

    def __init__(self):
        super().__init__('twist_controller')
        self._forward_speed = 0.5  # m/s, adjust as needed
        self._Kp = 5.5             # proportional gain, adjust as needed
        self._cmd_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', QoSPresetProfiles.SYSTEM_DEFAULT.value)
        self._error_sub = self.create_subscription(
            Float32,
            'colour_error',
            self._error_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
    def _error_callback(self, err):
        error = err.data
        msg = TwistStamped()
        
        # Set the header with current timestamp and frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        print(f"Received error: {error:.3f}")
        msg.twist.linear.x  = self._forward_speed      # constant cruise speed
        msg.twist.angular.z = -self._Kp * error        # proportional steering
        self._cmd_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = twistController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()