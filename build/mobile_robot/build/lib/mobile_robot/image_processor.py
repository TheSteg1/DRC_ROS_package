#!/usr/bin/env python3
"""
image_processor.py
------------------
Subscribes to camera/image_raw, applies HSV colour thresholding,
and publishes a normalised error signal suitable for a steering controller.

Error convention:
  positive → more target colour on the LEFT  → steer left
  negative → more target colour on the RIGHT → steer right
  zero     → balanced / target centred

Publishes a debug mask image so you can inspect the threshold live
in rqt_image_view without touching the node logic.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import cv2
import cv_bridge
import numpy as np


# ---------------------------------------------------------------------------
# Tunable HSV bounds — adjust these for your target colours.
# Tip: run `ros2 run rqt_image_view rqt_image_view` and pipe the raw image
# through a quick HSV picker script first to nail these values.
#
# OpenCV HSV ranges:  H: 0-179   S: 0-255   V: 0-255
# ---------------------------------------------------------------------------
DEFAULT_HSV_LOWER_YELLOW = (25, 50, 50)   # yellow lower bound (H, S, V)
DEFAULT_HSV_UPPER_YELLOW = (35, 255, 255)   # yellow upper bound
DEFAULT_HSV_LOWER_BLUE = (100, 50, 50)   # blue lower bound (H, S, V)
DEFAULT_HSV_UPPER_BLUE = (130, 255, 255)   # blue upper bound


class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        # --- Parameters (tunable at launch or runtime via ros2 param set) ---
        self.declare_parameter('hsv_lower_yellow', list(DEFAULT_HSV_LOWER_YELLOW))
        self.declare_parameter('hsv_upper_yellow', list(DEFAULT_HSV_UPPER_YELLOW))
        self.declare_parameter('hsv_lower_blue', list(DEFAULT_HSV_LOWER_BLUE))
        self.declare_parameter('hsv_upper_blue', list(DEFAULT_HSV_UPPER_BLUE))

        # Only look at the bottom fraction of the frame — the track marking
        # is usually at the bottom of the camera view, and cropping reduces
        # noise from the environment above the floor.
        self.declare_parameter('roi_fraction', 0.8)

        # Minimum pixel count to trust a detection.  Below this the track
        # is probably not visible and we should hold the last error rather
        # than commanding a wild turn.
        self.declare_parameter('min_pixel_count', 10)

        self._bridge = cv_bridge.CvBridge()
        self._last_error = 0.0   # held when track is lost

        # --- Subscribers ---
        # Use SENSOR_DATA QoS profile (best-effort, small queue) — matches
        # what camera drivers typically publish with.
        self._image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self._image_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # --- Publishers ---
        self._error_pub = self.create_publisher(
            Float32,
            'colour_error',
            10,
        )

        # Debug mask — subscribe with rqt_image_view to see the threshold
        self._mask_pub = self.create_publisher(
            Image,
            'debug/mask_image',
            10,
        )

        self.get_logger().info('image_processor started — waiting for images')

    # -----------------------------------------------------------------------
    # Main callback
    # -----------------------------------------------------------------------
    def _image_callback(self, msg: Image):

        # 1. ROS Image → OpenCV BGR
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # 2. Crop to region of interest (bottom N% of frame)
        roi_fraction = self.get_parameter('roi_fraction').value
        h, w = bgr.shape[:2]
        roi_top = int(h * (1.0 - roi_fraction))
        roi = bgr[roi_top:h, 0:w]

        # 3. BGR → HSV for colour thresholding
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 4. Build masks for yellow and blue from current parameter values
        lower_yellow = np.array(self.get_parameter('hsv_lower_yellow').value, dtype=np.uint8)
        upper_yellow = np.array(self.get_parameter('hsv_upper_yellow').value, dtype=np.uint8)
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_blue = np.array(self.get_parameter('hsv_lower_blue').value, dtype=np.uint8)
        upper_blue = np.array(self.get_parameter('hsv_upper_blue').value, dtype=np.uint8)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Optional: small morphological cleanup to remove salt-and-pepper noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN,  kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN,  kernel)
        blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

        # 5. Publish the debug mask (combine both for visualization)
        try:
            combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)
            mask_msg = self._bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
            mask_msg.header = msg.header   # preserve timestamp + frame_id
            self._mask_pub.publish(mask_msg)
        except cv_bridge.CvBridgeError as e:
            self.get_logger().warn(f'Could not publish debug mask: {e}')

        # 6. Compute steering error
        error = self._compute_error(yellow_mask, blue_mask, w)

        # 7. Publish
        error_msg = Float32()
        error_msg.data = float(error)
        self._error_pub.publish(error_msg)

    # -----------------------------------------------------------------------
    # Error computation
    # -----------------------------------------------------------------------
    def _compute_error(self, yellow_mask: np.ndarray, blue_mask: np.ndarray, image_width: int) -> float:
        """
        Find the centroid of yellow and blue masks, compute their midpoint,
        and determine how offset that midpoint is from the center of the image.
        
        Returns a value in [-1.0, 1.0]:
          +1.0 → midpoint at left edge
          -1.0 → midpoint at right edge
           0.0 → midpoint centered
        """
        min_pixels = self.get_parameter('min_pixel_count').value

        yellow_pixels = int(np.sum(yellow_mask > 0))
        blue_pixels = int(np.sum(blue_mask > 0))

        if yellow_pixels < min_pixels or blue_pixels < min_pixels:
            # Not enough pixels from either mask — hold last known error
            self.get_logger().warn(
                f'Insufficient pixels (yellow: {yellow_pixels}, blue: {blue_pixels}), '
                f'holding error={self._last_error:.3f}',
                throttle_duration_sec=2.0,
            )
            return self._last_error

        # Find centroids using moments
        yellow_moments = cv2.moments(yellow_mask)
        blue_moments = cv2.moments(blue_mask)

        if yellow_moments['m00'] == 0 or blue_moments['m00'] == 0:
            return self._last_error

        yellow_cx = yellow_moments['m10'] / yellow_moments['m00']
        blue_cx = blue_moments['m10'] / blue_moments['m00']

        # Compute midpoint between the two centroids
        midpoint = (yellow_cx + blue_cx) / 2.0

        # Determine offset from image center, normalized to [-1, 1]
        center = image_width / 2.0
        error = (midpoint - center) / center

        self._last_error = error
        return error


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
