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

from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, Bool

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
DEFAULT_HSV_LOWER_BLUE = (80, 0, 65)   # blue lower bound (H, S, V)
DEFAULT_HSV_UPPER_BLUE = (115, 255, 165)   # blue upper bound
DEFAULT_HSV_LOWER_YELLOW = (30, 65, 100)   # yellow lower bound (H, S, V)
DEFAULT_HSV_UPPER_YELLOW = (46, 255, 255)   # yellow upper bound

# Red wraps around H=0/179 in OpenCV, so we need two ranges and OR them.
DEFAULT_HSV_LOWER_RED1 = (0, 0, 0)
DEFAULT_HSV_UPPER_RED1 = (0, 0, 0)
DEFAULT_HSV_LOWER_RED2 = (0, 0, 0)
DEFAULT_HSV_UPPER_RED2 = (0, 0, 0)
# DEFAULT_HSV_LOWER_RED2 = (134, 0, 0)
# DEFAULT_HSV_UPPER_RED2 = (180, 255, 255)

# DEFAULT_HSV_LOWER_BLUE = (0, 0, 0)   # yellow lower bound (H, S, V)
# DEFAULT_HSV_UPPER_BLUE = (0, 0, 0 ) #ywlow upper bound
# DEFAULT_HSV_LOWER_YELLOW = (0, 0, 0)   # blue lower bound (H, S, V)
# DEFAULT_HSV_UPPER_YELLOW = (0, 0, 0)   # blue upper bound

# # Red wraps around H=0/179 in OpenCV, so we need two ranges and OR them.
# DEFAULT_HSV_LOWER_RED1 = (0, 0, 0)
# DEFAULT_HSV_UPPER_RED1 = (0, 0, 0)
# DEFAULT_HSV_LOWER_RED2 = (0, 0, 0)
# DEFAULT_HSV_UPPER_RED2 = (0, 0, 0)



class ImageProcessor(Node):

    def __init__(self):
        super().__init__('image_processor')

        # --- Parameters (tunable at launch or runtime via ros2 param set) ---
        self.declare_parameter('hsv_lower_yellow', list(DEFAULT_HSV_LOWER_YELLOW))
        self.declare_parameter('hsv_upper_yellow', list(DEFAULT_HSV_UPPER_YELLOW))
        self.declare_parameter('hsv_lower_blue', list(DEFAULT_HSV_LOWER_BLUE))
        self.declare_parameter('hsv_upper_blue', list(DEFAULT_HSV_UPPER_BLUE))

        # Obstacle (red) params
        self.declare_parameter('hsv_lower_red1', list(DEFAULT_HSV_LOWER_RED1))
        self.declare_parameter('hsv_upper_red1', list(DEFAULT_HSV_UPPER_RED1))
        self.declare_parameter('hsv_lower_red2', list(DEFAULT_HSV_LOWER_RED2))
        self.declare_parameter('hsv_upper_red2', list(DEFAULT_HSV_UPPER_RED2))

        # -------Undistort the image using camera intrinsics from calibration------- #
        self.K = np.array([
            [216.51078, -0.92533, 319.68753],
            [0.0,       218.64979, 238.17170],
            [0.0,       0.0,       1.0]
        ], dtype=np.float64)

        self.D = np.array([
            0.248201,
        -1.308089,
            1.614096,
        -0.372616
        ], dtype=np.float64)

        self.image_size = (640, 480)

        self.map1, self.map2 = cv2.fisheye.initUndistortRectifyMap(
            self.K,
            self.D,
            np.eye(3),
            self.K,
            self.image_size,
            cv2.CV_16SC2
        )
        # ----------------------------------------------------------------------- #

        # Only look at the bottom fraction of the frame — the track marking
        # is usually at the bottom of the camera view, and cropping reduces
        # noise from the environment above the floor.
        self.declare_parameter('roi_fraction', 0.9)

        # Minimum pixel count to trust a detection.  Below this the track
        # is probably not visible and we should hold the last error rather
        # than commanding a wild turn.
        self.declare_parameter('min_pixel_count', 200)

        # Minimum red pixel count to declare an obstacle present.
        # Set this high enough to reject stray red pixels in the environment.
        self.declare_parameter('obstacle_min_pixels', 200)


        self._bridge = cv_bridge.CvBridge()
        self._last_error = 0.0   # held when track is lost

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

        # --- Publishers ---
        self._error_pub = self.create_publisher(
            Float32,
            'colour_error',
            10,
        )
        self.lanes_visible_pub = self.create_publisher(
            Bool,
            'lanes_visible',
            10,
        )

        self._obstacle_pub = self.create_publisher(Float32, 'obstacle_info', 10)
        # Debug mask — subscribe with rqt_image_view to see the threshold
        self._mask_pub = self.create_publisher(
            Image,
            'debug/mask_image',
            10,
        )
        self._yellow_mask_pub = self.create_publisher(Image, '/vision/hsv_mask/yellow', 10)
        self._blue_mask_pub = self.create_publisher(Image, '/vision/hsv_mask/blue', 10)
        self._obstacle_size_pub = self.create_publisher(Float32, 'obstacle_size', 10)
        self.get_logger().info('image_processor started — waiting for images')


    def _publish_colour_outputs(self, mask, header, mask_pub):
        mask_msg = self._bridge.cv2_to_imgmsg(mask, encoding='mono8')
        mask_msg.header = header
        mask_pub.publish(mask_msg)

    # -----------------------------------------------------------------------
    # Main callback
    # -----------------------------------------------------------------------
    def _image_callback(self, msg: Image):
        try:
            bgr = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return
        
        # Undistort the image using precomputed maps from camera calibration
        # bgr = cv2.remap(
        #     bgr,
        #     self.map1,
        #     self.map2,
        #     interpolation=cv2.INTER_LINEAR,
        #     borderMode=cv2.BORDER_CONSTANT
        # )
 
        roi_fraction = self.get_parameter('roi_fraction').value
        h, w = bgr.shape[:2]
        roi_top = int(h * (1.0 - roi_fraction))
        roi = bgr[roi_top:h, 0:w]
        #blurred = cv2.GaussianBlur(roi, (3,3), 0)
 
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
 
        # --- Lane masks ---
        def make_mask(lower_param, upper_param):
            lo = np.array(self.get_parameter(lower_param).value, dtype=np.uint8)
            hi = np.array(self.get_parameter(upper_param).value, dtype=np.uint8)
            m  = cv2.inRange(hsv, lo, hi)
            m  = cv2.morphologyEx(m, cv2.MORPH_CLOSE,  kernel)
            #m  = cv2.erode(m, kernel, iterations=2)
            m  = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
            
            m  = self._filter_line_contours(m)   # <-- keep only long/thin shapes
            return m
 
        yellow_mask = make_mask('hsv_lower_yellow', 'hsv_upper_yellow')
        blue_mask   = make_mask('hsv_lower_blue',   'hsv_upper_blue')

        self._publish_colour_outputs(
            yellow_mask,
            msg.header,
            self._yellow_mask_pub,
        )

        self._publish_colour_outputs(
            blue_mask,
            msg.header,
            self._blue_mask_pub,
        )
 
        # --- Obstacle mask (red wraps around hue, so combine two ranges) ---
        # lo1 = np.array(self.get_parameter('hsv_lower_red1').value, dtype=np.uint8)
        # hi1 = np.array(self.get_parameter('hsv_upper_red1').value, dtype=np.uint8)
        # lo2 = np.array(self.get_parameter('hsv_lower_red2').value, dtype=np.uint8)
        # hi2 = np.array(self.get_parameter('hsv_upper_red2').value, dtype=np.uint8)
        # red_mask = cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1),
        #                           cv2.inRange(hsv, lo2, hi2))
        # red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)
        # red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
 
        # --- Debug: publish colour-coded BGR overlay ---
        # try:
        #     debug_bgr = roi.copy()
        #     debug_bgr[yellow_mask > 0] = (0, 255, 255)   # yellow tint
        #     debug_bgr[blue_mask   > 0] = (255, 100,  0)  # blue tint
        #     debug_bgr[red_mask    > 0] = (0,   0, 255)   # red tint
        #     dbg_msg = self._bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8')
        #     dbg_msg.header = msg.header
        #     self._mask_pub.publish(dbg_msg)
        # except cv_bridge.CvBridgeError as e:
        #     self.get_logger().warn(f'Could not publish debug image: {e}')


        # --- Publish lane error ---
        error, lanes_visible, midpoint_x = self._compute_lane_error(yellow_mask, blue_mask, w)

        try:
            #combined_mask = cv2.bitwise_or(cv2.bitwise_or(yellow_mask, blue_mask), red_mask)
            combined_mask = cv2.bitwise_or(yellow_mask, blue_mask)

            # Convert to BGR so we can draw colored markers
            debug_img = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)

            # Draw the image center line (reference) in white
            center_x = w // 2
            cv2.line(debug_img, (center_x, 0), (center_x, debug_img.shape[0]),
                    (255, 255, 255), 1)

            # Draw the midpoint marker if lanes are visible
            if midpoint_x is not None:
                mx = int(midpoint_x)
                my = debug_img.shape[0] // 2  # vertical center of the ROI
                cv2.drawMarker(debug_img, (mx, my), (0, 255, 0),
                                markerType=cv2.MARKER_CROSS,
                                markerSize=20, thickness=2)
                cv2.circle(debug_img, (mx, my), 6, (0, 255, 0), 2)

            mask_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            mask_msg.header = msg.header
            self._mask_pub.publish(mask_msg)
            # mask_msg = self._bridge.cv2_to_imgmsg(combined_mask, encoding='mono8')
            # mask_msg.header = msg.header   # preserve timestamp + frame_id
            # self._mask_pub.publish(mask_msg)
        except cv_bridge.CvBridgeError as e:
            self.get_logger().warn(f'Could not publish debug mask: {e}')

        error_msg = Float32()
        error_msg.data = float(error)
        self._error_pub.publish(error_msg)

        lanes_visible_msg = Bool()
        lanes_visible_msg.data = lanes_visible
        self.lanes_visible_pub.publish(lanes_visible_msg)
        
 
        # --- Publish obstacle info ---
        # offset, norm_size = self._compute_obstacle_offset(red_mask, w)  # offset in [-1, 1], norm_size in [0, 1]
        offset = 0.0
        norm_size = 0.0

        obs_msg = Float32()
        obs_msg.data = float(offset)
        self._obstacle_pub.publish(obs_msg)

        size_msg = Float32()
        size_msg.data = float(norm_size)
        self._obstacle_size_pub.publish(size_msg)

    # -----------------------------------------------------------------------
    # Filter for lines
    # -----------------------------------------------------------------------
    def _filter_line_contours(self, mask, min_area=100, min_aspect=3.0, min_extent=0.2, max_extent=1):
        """
        Keep only contours that are long and thin (lane markings), and drop
        blobby/short noise. Returns a new mask containing just the filtered
        contours, filled in.
        """

        # def auto_canny(image, sigma=0.33):
        #     v = np.median(mask)

        #     lower = int(max(0,(1.0-sigma)*v))
        #     upper = int(min(255,(1.0+sigma)*v))
        #     edged = cv2.Canny(image,lower,upper)

        #     return edged
    
        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        # filtered = auto_canny(mask)

        

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        final_edges = np.zeros_like(mask)

        for c in contours:
             area = cv2.contourArea(c)
             if area < min_area:
                 continue

             x, y, bw, bh = cv2.boundingRect(c)
             if bw * bh == 0:
                 continue
             extent = area / float(bw * bh)   # how much of its bbox it fills

             rect = cv2.minAreaRect(c)        # rotated rect, handles diagonal lines
             rw, rh = rect[1]
             if min(rw, rh) == 0:
                continue
             aspect = max(rw, rh) / min(rw, rh)

             if  extent <= max_extent and aspect >= min_aspect: 
                 cv2.drawContours(final_edges, [c], -1, 255, thickness=cv2.FILLED)
        return final_edges
    # -----------------------------------------------------------------------
    # Error computation
    # -----------------------------------------------------------------------
    def _compute_lane_error(self, yellow_mask: np.ndarray, blue_mask: np.ndarray, image_width: int) -> tuple[float, bool]:
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
            if yellow_pixels < min_pixels:
                self._last_error = -0.08  # bias right if we lose the left lane
                pass
            elif blue_pixels < min_pixels:
                self._last_error = 0.08   # bias left if we lose the right lane
                pass
            return self._last_error, False, None  # lanes not visible



        # Find centroids using moments
        yellow_moments = cv2.moments(yellow_mask)
        blue_moments = cv2.moments(blue_mask)

        if yellow_moments['m00'] == 0 or blue_moments['m00'] == 0:
            return self._last_error, False, None  # lanes not visible

        yellow_cx = yellow_moments['m10'] / yellow_moments['m00']
        blue_cx = blue_moments['m10'] / blue_moments['m00']

        # Compute midpoint between the two centroids
        midpoint = (yellow_cx + blue_cx) / 2.0

        # Determine offset from image center, normalized to [-1, 1]
        center = image_width / 2.0
        error = (midpoint - center) / center

        self._last_error = error
        return error, True, midpoint

    # -----------------------------------------------------------------------
 
    def _compute_obstacle_offset(self, red_mask, image_width) -> tuple[float, float]:
        """
        Returns (normalised_lateral_offset, normalised_area).
        normalised_area = pixel_count / total_image_pixels → [0, 1]
        ~0.0  → obstacle far away or not visible
        ~0.05 → getting close   (good trigger point)
        ~0.15 → very close
        """
        min_pixels  = self.get_parameter('obstacle_min_pixels').value
        total_pixels = red_mask.shape[0] * red_mask.shape[1]
        pixel_count  = int(np.sum(red_mask > 0))

        if pixel_count < min_pixels:
            return 0.0, 0.0

        moments = cv2.moments(red_mask)
        if moments['m00'] == 0:
            return 0.0, 0.0

        cx     = moments['m10'] / moments['m00']
        center = image_width / 2.0
        offset = (cx - center) / center

        norm_size = pixel_count / float(total_pixels)   # normalised area

        return offset, norm_size


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
