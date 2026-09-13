#!/usr/bin/env python3
"""
MLX90640 -> ROS 2. The camera side of the thermal drop mission, and nothing else.

Wraps homography_mlx_rgb/thermal_image.py's MLX90640Reader (retry/backoff on
I2C errors, background acquisition thread) and publishes every NEW frame:

    thermal/image     sensor_msgs/Image, 32FC1, 32 wide x 24 high, degrees C.
                      header.stamp is the wall-clock time the frame was read,
                      which is what thermal_drop uses to pick the attitude
                      the frame was taken at.
    thermal/hotspot   std_msgs/String  max_temp|row|col|ambient  (bench aid)
    thermal/preview   sensor_msgs/Image, bgr8, upscaled inferno colormap.
                      Only if publish_preview is true -- it costs CPU.

Kept as its own process on purpose, like bar_detect: the I2C bus is slow and
occasionally stalls under vibration, and none of that may ever sit on the
thread that produces the offboard heartbeat.

Bench:
    ros2 run drone_testing thermal_sensor
    ros2 topic echo /thermal/hotspot
"""

import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

H, W = 24, 32


class ThermalSensor(Node):

    def __init__(self):
        super().__init__('thermal_sensor')

        self.refresh_hz = int(self.declare_parameter('refresh_hz', 8).value)
        self.publish_preview = bool(self.declare_parameter('publish_preview', False).value)
        self.preview_scale = int(self.declare_parameter('preview_scale', 20).value)

        # Imported here, not at module level, so a missing Blinka install fails
        # with a clear message instead of an import trace from setup's entry point.
        import adafruit_mlx90640
        from homography_mlx_rgb.thermal_image import MLX90640Reader

        rates = {
            1: adafruit_mlx90640.RefreshRate.REFRESH_1_HZ,
            2: adafruit_mlx90640.RefreshRate.REFRESH_2_HZ,
            4: adafruit_mlx90640.RefreshRate.REFRESH_4_HZ,
            8: adafruit_mlx90640.RefreshRate.REFRESH_8_HZ,
            16: adafruit_mlx90640.RefreshRate.REFRESH_16_HZ,
        }
        if self.refresh_hz not in rates:
            self.get_logger().warning(
                f"refresh_hz {self.refresh_hz} is not one of {sorted(rates)}; using 4.")
            self.refresh_hz = 4

        self.reader = MLX90640Reader(refresh_rate=rates[self.refresh_hz])
        self.reader.start()

        self.image_pub = self.create_publisher(Image, 'thermal/image', 10)
        self.hotspot_pub = self.create_publisher(String, 'thermal/hotspot', 10)
        self.preview_pub = (self.create_publisher(Image, 'thermal/preview', 10)
                            if self.publish_preview else None)

        self.last_ts = None
        self.frames = 0
        self.started = time.monotonic()
        self.create_timer(0.02, self.poll)

        self.get_logger().warning(
            f"MLX90640 up at {self.refresh_hz} Hz. Publishing thermal/image "
            "(32FC1, deg C).")

    def poll(self):
        frame = self.reader.get_latest_frame()
        if frame is None:
            if time.monotonic() - self.started > 5.0 and self.frames == 0:
                self.get_logger().warning(
                    "No MLX90640 frame yet -- check the I2C wiring.",
                    throttle_duration_sec=5.0)
            return
        if frame.timestamp == self.last_ts:
            return
        self.last_ts = frame.timestamp
        self.frames += 1

        grid = np.asarray(frame.grid, dtype=np.float32).reshape(H, W)
        stamp = Time()
        stamp.sec = int(frame.timestamp)
        stamp.nanosec = int((frame.timestamp - int(frame.timestamp)) * 1e9)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'thermal_camera'
        msg.height, msg.width = H, W
        msg.encoding = '32FC1'
        msg.is_bigendian = 0
        msg.step = W * 4
        msg.data = grid.tobytes()
        self.image_pub.publish(msg)

        ambient = float(np.median(grid))
        row, col = frame.max_pixel
        self.hotspot_pub.publish(String(
            data=f"{frame.max_temp:.2f}|{row}|{col}|{ambient:.2f}"))

        if self.preview_pub is not None:
            self._publish_preview(grid, stamp)

    def _publish_preview(self, grid, stamp):
        import cv2
        lo, hi = float(grid.min()), float(grid.max())
        if hi - lo < 0.1:
            hi = lo + 0.1
        norm = np.uint8((grid - lo) * 255.0 / (hi - lo))
        img = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)
        img = cv2.resize(img, (W * self.preview_scale, H * self.preview_scale),
                         interpolation=cv2.INTER_NEAREST)
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'thermal_camera'
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        self.preview_pub.publish(msg)

    def destroy_node(self):
        try:
            self.reader.close()
        except Exception as exc:
            self.get_logger().warning(f"Reader close failed: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalSensor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
