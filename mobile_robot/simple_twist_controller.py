#!/usr/bin/env python3

# ros2 run mobile_robot simple_twist_controller --ros-args --params-file /ros2_ws/src/mobile_robot/parameters/testing_simple_image_processor.yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

import numpy as np
import cv2
import time
import message_filters
from cv_bridge import CvBridge

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image


class TwistController(Node):

    def __init__(self):
        super().__init__('simple_twist_controller')

        ##### CONTROL CONFIG
        self.declare_parameter('bang_bang_inset', 0.1)
        self.bang_bang_inset = self.get_parameter('bang_bang_inset').value
        
        self.declare_parameter('turn_time', 0.1)
        self.turn_time = self.get_parameter('turn_time').value

        # Watchdog: if no message arrives within this many seconds, stop.
        self.declare_parameter('watchdog_timeout', 0.5)


        # --- Subscribers ---

        self._enable_sub = self.create_subscription(
            Bool, 'enable_control',
            self._enable_callback,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._enabled = False
        self._bridge = CvBridge()
        
        # match pairs of blue and yellow masks and run control loop
        self.blue_sub = message_filters.Subscriber(self, Image, '/vision/roi/blue')
        self.yellow_sub = message_filters.Subscriber(self, Image, '/vision/roi/yellow')

        # Adjust slop (seconds) based on your pipeline's latency
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.blue_sub, self.yellow_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.monitor_images_loop)

        # --- Publisher ---
        self._cmd_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        
        
        ##### DEBUG PUBLISHERS
        
        self.declare_parameter('CONFIG_DEBUG', False)
        self.CONFIG_DEBUG = self.get_parameter('CONFIG_DEBUG').value
        
        self.debug_decision_making_pub = self.create_publisher(
            Image,
            "/debug/decision_making",
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        
        
        ### State machine stuff
        
        # create a timer that calls back state_machine 
        self.state_machine_timer = self.create_timer(0.1, self.state_machine) # 10 Hz
        
        self.state = "GO_STRAIGHT"  # initial state
        self.state_event_time = time.monotonic()

        self.get_logger().info(
            'twist_controller started — DISABLED by default. '
            'Publish True to enable_control to start driving.'
        )



    """
    Control Logic:
    - FREQUENCY: This function is called whenever there is a new pair of blue and yellow masks available.


    
    """
    def monitor_images_loop(self, blue_mask, yellow_mask):
        
        blue_mask_cv   = self._bridge.imgmsg_to_cv2(blue_mask,   desired_encoding='mono8')
        yellow_mask_cv = self._bridge.imgmsg_to_cv2(yellow_mask, desired_encoding='mono8')

        h, w = blue_mask_cv.shape
        blue_bang_bang_line, yellow_bang_bang_line = self.get_bang_bang_lines(blue_mask)

        blue_x_indices   = np.nonzero(blue_mask_cv)[1]
        yellow_x_indices = np.nonzero(yellow_mask_cv)[1]        

        left_most_blue_x    = int(blue_x_indices.min())   if blue_x_indices.size   > 0 else None
        right_most_yellow_x = int(yellow_x_indices.max()) if yellow_x_indices.size > 0 else None    
        
        if self.CONFIG_DEBUG:
            # Combine masks in grayscale, then convert to color so line colors are visible.
            combined = cv2.bitwise_or(blue_mask_cv, yellow_mask_cv)
            combined_bgr = cv2.cvtColor(combined, cv2.COLOR_GRAY2BGR)

            # Draw vertical bang-bang guide lines in yellow and blue.
            cv2.line(combined_bgr, (yellow_bang_bang_line, 0), (yellow_bang_bang_line, h), (0, 255, 255), 4)
            cv2.line(combined_bgr, (blue_bang_bang_line, 0), (blue_bang_bang_line, h), (255, 0, 0), 4)
            
            # put line on left most and right most
            
            if left_most_blue_x is not None:
                cv2.line(combined_bgr, (left_most_blue_x, 0), (left_most_blue_x, h), (0, 0, 255), 3)
            if right_most_yellow_x is not None:
                cv2.line(combined_bgr, (right_most_yellow_x, 0), (right_most_yellow_x, h), (0, 0, 255), 3)

            debug_msg = self._bridge.cv2_to_imgmsg(combined_bgr, encoding='bgr8')
            debug_msg.header = blue_mask.header
            self.debug_decision_making_pub.publish(debug_msg)
            
        # Only set new bang state if we're in a stable/receptive state
        if self.state in (None, "GO_STRAIGHT", "GO_STRAIGHT_SETTLING"):   
            if left_most_blue_x is not None and left_most_blue_x < blue_bang_bang_line:
                self.state = "BANG_LEFT"
            
            elif right_most_yellow_x is not None and right_most_yellow_x > yellow_bang_bang_line:
                self.state = "BANG_RIGHT"
                                
    def state_machine(self):
        
        if not self._enabled:
            return
        
        now = time.monotonic()
        
        match self.state:
            case "BANG_LEFT":
                self.get_logger().info("BANG LEFT")
                self.publish_movement(linear_x=0.0, angular_z=0.05)
                self.state_event_time = now + self.turn_time
                self.state = "BANG_LEFT_SETTLING"
                
            case "BANG_LEFT_SETTLING": 
                if now >= self.state_event_time:
                    self.state = "GO_STRAIGHT"
            
            case "BANG_RIGHT":
                self.get_logger().info("BANG RIGHT")
                self.publish_movement(linear_x=0.0, angular_z=-0.05)
                self.state_event_time = now + self.turn_time
                self.state = "BANG_RIGHT_SETTLING"
                
            case "BANG_RIGHT_SETTLING":
                if now >= self.state_event_time:
                    self.state = "GO_STRAIGHT"
                    
            case "GO_STRAIGHT":
                self.get_logger().info("GO STRAIGHT")
                self.publish_movement(linear_x=0.1, angular_z=0.0)
                self.state = "GO_STRAIGHT_SETTLING"
                
            case "GO_STRAIGHT_SETTLING":
                pass                        
                
    def get_bang_bang_lines(self, blue_mask):
        # get number of pixels in each mask (blue_mask)
        h, w = blue_mask.height, blue_mask.width
        
        # calculate the inset from bang_bang_inset
        
        bang_bang_inset_pixels = int(self.bang_bang_inset * w)
        
        yellow_bang_bang_line = 0 + bang_bang_inset_pixels
        blue_bang_bang_line = w - bang_bang_inset_pixels
        return blue_bang_bang_line, yellow_bang_bang_line

    def _enable_callback(self, msg: Bool):
        if msg.data != self._enabled:
            self.get_logger().info(
                f"Control {'ENABLED' if msg.data else 'DISABLED'} via enable_control topic"
            )
        self._enabled = msg.data
        if not self._enabled:
            # Immediately publish a stop the moment we're disabled, rather
            # than waiting for the next control loop tick.
            self.publish_movement(0.0, 0.0)

    def publish_movement(self, linear_x: float, angular_z: float):
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