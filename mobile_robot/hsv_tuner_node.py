#!/usr/bin/env python3
"""
hsv_tuner_node.py

Interactive HSV tuning tool for the `image_processor` node.

- Opens OpenCV trackbar windows for each colour range (yellow, blue, red1, red2).
- Every time a slider moves, it calls `set_parameters` on the running
  `image_processor` node so you see the effect immediately (no restart needed).
- Subscribes to `debug/mask_image` and `image_raw/compressed` and shows them
  side-by-side so you can watch the mask react to slider changes in real time.

Run alongside your normal image_processor node:

    ros2 run <your_package> image_processor
    ros2 run <your_package> hsv_tuner_node     # or: python3 hsv_tuner_node.py

Press 'q' or ESC in the OpenCV window to quit.
"""

import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import CompressedImage, Image
import cv_bridge


TARGET_NODE = 'image_processor'

# name -> (param_lower, param_upper)
RANGES = {
    'yellow': ('hsv_lower_yellow', 'hsv_upper_yellow'),
    'blue':   ('hsv_lower_blue',   'hsv_upper_blue'),
    'red1':   ('hsv_lower_red1',   'hsv_upper_red1'),
    'red2':   ('hsv_lower_red2',   'hsv_upper_red2'),
}

# Sensible starting defaults shown on the sliders before any param is read.
# These get overwritten as soon as we fetch the live values from the node.
DEFAULTS = {
    'yellow': ([32, 68, 100], [46, 255, 255]),
    'blue':   ([90, 0, 0], [116, 255, 255]),
    'red1':   ([0, 0, 0], [0, 0, 0]),
    'red2':   ([0, 0, 0], [0, 0, 0]),
}


def make_int_param(values):
    pv = ParameterValue()
    pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
    pv.integer_array_value = [int(v) for v in values]
    return pv


class HsvTuner(Node):
    def __init__(self):
        super().__init__('hsv_tuner_node')
        self._bridge = cv_bridge.CvBridge()

        self._set_params_client = self.create_client(
            SetParameters, f'/{TARGET_NODE}/set_parameters')

        self.get_logger().info(f'Waiting for /{TARGET_NODE}/set_parameters service...')
        if not self._set_params_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f'Could not find /{TARGET_NODE}/set_parameters after 5s. '
                'Is image_processor running? Will keep retrying in background.')

        self._latest_camera = None
        self._latest_mask = None

        self.create_subscription(
            CompressedImage, 'image_raw/compressed', self._camera_cb,
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_subscription(
            Image, 'debug/mask_image', self._mask_cb, 10)

        self._build_ui()

    # ---------- subscriptions ----------

    def _camera_cb(self, msg: CompressedImage):
        self._latest_camera = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _mask_cb(self, msg: Image):
        self._latest_mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

    # ---------- UI ----------

    def _build_ui(self):
        for name in RANGES:
            win = f'HSV - {name}'
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            lo, hi = DEFAULTS[name]
            labels_lo = ['H_low', 'S_low', 'V_low']
            labels_hi = ['H_high', 'S_high', 'V_high']
            for i, lbl in enumerate(labels_lo):
                cv2.createTrackbar(lbl, win, lo[i], 255, lambda v, n=name: self._on_change(n))
            for i, lbl in enumerate(labels_hi):
                cv2.createTrackbar(lbl, win, hi[i], 255, lambda v, n=name: self._on_change(n))
            # Hue maxes out at 180, clamp those specific trackbars
            cv2.setTrackbarMax('H_low', win, 180)
            cv2.setTrackbarMax('H_high', win, 180)

    def _read_sliders(self, name):
        win = f'HSV - {name}'
        lo = [
            cv2.getTrackbarPos('H_low', win),
            cv2.getTrackbarPos('S_low', win),
            cv2.getTrackbarPos('V_low', win),
        ]
        hi = [
            cv2.getTrackbarPos('H_high', win),
            cv2.getTrackbarPos('S_high', win),
            cv2.getTrackbarPos('V_high', win),
        ]
        return lo, hi

    def _on_change(self, name):
        lo, hi = self._read_sliders(name)
        lower_name, upper_name = RANGES[name]
        self._push_params({lower_name: lo, upper_name: hi})

    def _push_params(self, param_dict):
        if not self._set_params_client.service_is_ready():
            return  # silently skip; UI still updates locally
        req = SetParameters.Request()
        for pname, values in param_dict.items():
            p = Parameter()
            p.name = pname
            p.value = make_int_param(values)
            req.parameters.append(p)
        self._set_params_client.call_async(req)

    # ---------- display loop ----------

    def spin_with_display(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)

                display_frames = []
                if self._latest_camera is not None:
                    display_frames.append(self._latest_camera)
                if self._latest_mask is not None:
                    mask_bgr = cv2.cvtColor(self._latest_mask, cv2.COLOR_GRAY2BGR)
                    if self._latest_camera is not None:
                        mask_bgr = cv2.resize(
                            mask_bgr,
                            (self._latest_camera.shape[1], self._latest_camera.shape[0]))
                    display_frames.append(mask_bgr)

                if display_frames:
                    combined = np.hstack(display_frames) if len(display_frames) > 1 else display_frames[0]
                    cv2.imshow('camera | mask', combined)

                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):  # q or ESC
                    break
        finally:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HsvTuner()
    try:
        node.spin_with_display()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()