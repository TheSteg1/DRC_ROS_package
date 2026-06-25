#!/usr/bin/env python3
# literally publish masks


# source /ros2_ws/install/setup.bash && ros2 run mobile_robot simple_image_processor --ros-args --params-file /ros2_ws/src/mobile_robot/parameters/testing_simple_image_processor.yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Bool

import cv2
import cv_bridge
import numpy as np


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        # --- Parameters (tunable at launch or runtime via ros2 param set) ---
        # OpenCV HSV ranges: H: 0-179, S: 0-255, V: 0-255
        self.declare_parameter('hsv_lower_yellow', [30, 65, 100])
        self.declare_parameter('hsv_upper_yellow', [46, 255, 255])
        self.declare_parameter('hsv_lower_blue', [80, 0, 65])
        self.declare_parameter('hsv_upper_blue', [115, 255, 165])


        self.image_size = (640, 480)
        
        self.roi_percentage = self.declare_parameter('roi_percentage', 0.5)


        self._bridge = cv_bridge.CvBridge()

        # --- Subscribers ---
        # Use SENSOR_DATA QoS profile (best-effort, small queue) — matches
        # what camera drivers typically publish with.
        self._image_sub = self.create_subscription(
            CompressedImage,
            'image_raw/compressed', #USE FOR REAL
            #'camera/image_raw/compressed', #USE FOR SIM
            self._image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )


        self.yellow = []
        self.blue = []
        
        self._yellow_mask_pub = self.create_publisher(Image, '/vision/hsv_mask/yellow', 10)
        self._blue_mask_pub = self.create_publisher(Image, '/vision/hsv_mask/blue', 10)
        self.roi_yellow = self.create_publisher(Image, '/vision/roi/yellow', 10)
        self.roi_blue = self.create_publisher(Image, '/vision/roi/blue', 10)
        self.get_logger().info('image_processor started — waiting for images')


    def _publish_colour_outputs(self, mask, header, mask_pub):
        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = header
        mask_pub.publish(mask_msg)

    # -----------------------------------------------------------------------
    # Main callback
    # -----------------------------------------------------------------------
    def _image_callback(self, msg: CompressedImage):
        try:
            bgr = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
 
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        def make_mask(lower_param, upper_param):
            lo = np.array(self.get_parameter(lower_param).value, dtype=np.uint8)
            hi = np.array(self.get_parameter(upper_param).value, dtype=np.uint8)
            m = cv2.inRange(hsv, lo, hi)
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kernel)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
            return m
 
        self.yellow_mask = make_mask('hsv_lower_yellow', 'hsv_upper_yellow')
        self.blue_mask = make_mask('hsv_lower_blue', 'hsv_upper_blue')

        self._publish_colour_outputs(
            self.yellow_mask,
            msg.header,
            self._yellow_mask_pub,
        )

        self._publish_colour_outputs(
            self.blue_mask,
            msg.header,
            self._blue_mask_pub,
        )
        
        # filter for bottom half of image
        h, w = self.yellow_mask.shape
        
        rounded_bottom_cutoff = int(h * (1-self.roi_percentage.value)) 
        
        roi_yellow = np.zeros((h, w), dtype=np.uint8)
        roi_blue = np.zeros((h, w), dtype=np.uint8)
        roi_yellow[rounded_bottom_cutoff:, :] = self.yellow_mask[rounded_bottom_cutoff:, :]
        roi_blue[rounded_bottom_cutoff:, :] = self.blue_mask[rounded_bottom_cutoff:, :]
        self._publish_colour_outputs(
            roi_yellow,
            msg.header,
            self.roi_yellow,
        )
        self._publish_colour_outputs(
            roi_blue,
            msg.header,
            self.roi_blue,
        )


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
