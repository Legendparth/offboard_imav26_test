#!/usr/bin/env python3
"""
The SIMULATOR's stand-in for thermal_sensor.py: a Gazebo thermal camera ->
the same /thermal/image the flight node already knows how to read.

    thermal/raw       in   sensor_msgs/Image, mono16 (Gazebo L16), 32x24,
                           raw counts of KELVIN / resolution
    thermal/image     out  sensor_msgs/Image, 32FC1, 32x24, degrees C
    thermal/hotspot   out  std_msgs/String  max_temp|row|col|ambient
    thermal/preview   out  sensor_msgs/Image, bgr8, annotated. Off by default.
                           http://localhost:8082/ is the cheap way to watch it.

WHY THIS NODE EXISTS AT ALL

    thermal_sensor.py opens an MLX90640 over I2C. There is no I2C in a
    simulation, so something has to produce /thermal/image from what Gazebo
    renders -- and it has to produce it in EXACTLY the shape thermal_sensor
    does, because the flight node is not told which one it is talking to.
    Same topic, same 32FC1 encoding, same 32x24, same degrees C, same
    hotspot line, same annotated stream on the same port, drawn by the same
    find_blobs() from thermal_common. Swap this node for that one and the
    mission cannot tell.

THE CONVERSION, AND THE ONE NUMBER THAT MUST MATCH THE SDF

    The gz-sim-thermal-sensor-system plugin in thermal_cam.xacro quantises
    temperature into 16-bit counts:

        count = kelvin / resolution         (resolution = 0.01 K per count)

    so this node inverts it:

        degrees C = count * resolution - 273.15

    resolution here and <resolution> there are the SAME NUMBER. Change one
    and every temperature in the mission is wrong by a factor, which shows
    up as "no warm box found in the survey" (everything below min_contrast)
    or as every pixel in the arena being a blob. They are both parameters so
    that the pairing is visible; they are not independent.

WHY THE TIMESTAMP IS REWRITTEN

    thermal_drop matches each frame against a buffer of attitudes stamped
    with time.time() -- the WALL clock -- because on the aircraft the MLX's
    own frame timestamps are wall-clock. The image arriving here from the
    ros_gz bridge is stamped with Gazebo's SIMULATION clock, which starts at
    zero. Passing that through would make every frame look about
    fifty-seven years stale, _pose_at() would reject all of them, and the
    survey would map nothing while the camera worked perfectly. So the
    outgoing stamp is time.time(), matching the hardware node exactly.

    That is also why frame_latency stays useful: it is a wall-clock lag, and
    both clocks are now wall-clock.

Bench:
    ros2 run drone_testing thermal_sim --ros-args -p raw_topic:=/thermal/raw
    ros2 topic echo /thermal/hotspot
"""

import threading
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Image
from std_msgs.msg import String

from drone_testing.thermal_common import H, W, annotate, find_blobs
from drone_testing.thermal_sensor import MjpegServer


class ThermalSim(Node):

    def __init__(self):
        super().__init__('thermal_sim')

        self.raw_topic = str(self.declare_parameter(
            'raw_topic', 'thermal/raw').value)
        # KELVIN PER COUNT. Must equal <resolution> in thermal_cam.xacro.
        self.resolution = float(self.declare_parameter('resolution', 0.01).value)
        self.min_contrast = float(self.declare_parameter('min_contrast', 3.0).value)
        self.min_blob_pixels = int(self.declare_parameter('min_blob_pixels', 2).value)
        # Sensor noise, degrees C, 1 sigma, added per pixel. The MLX90640's
        # NETD is about 0.1 C and the whole blob/cluster/verify apparatus
        # exists because of it, so a simulation with a PERFECTLY clean frame
        # does not exercise the thing it is there to test. 0 turns it off.
        self.noise_c = float(self.declare_parameter('noise_c', 0.1).value)
        self.preview_scale = int(self.declare_parameter('preview_scale', 20).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 70).value)
        publish_preview = bool(self.declare_parameter('publish_preview', False).value)
        stream_port = int(self.declare_parameter('stream_port', 8082).value)

        self.image_pub = self.create_publisher(Image, 'thermal/image', 5)
        self.hotspot_pub = self.create_publisher(String, 'thermal/hotspot', 10)
        self.preview_pub = (self.create_publisher(Image, 'thermal/preview', 1)
                            if publish_preview else None)

        # The camera comes over the ros_gz bridge, which publishes BEST_EFFORT.
        # A RELIABLE subscription would never match it and this node would sit
        # there reporting no frames while gz happily rendered them.
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE,
                                history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, self.raw_topic, self.raw_callback,
                                 sensor_qos)

        self.frames = 0
        self.warned_shape = False
        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self._rng = np.random.default_rng(0)

        self.stream = None
        if stream_port > 0:
            try:
                self.stream = MjpegServer(self, stream_port)
            except OSError as exc:
                self.get_logger().error(
                    f"Could not start the stream on port {stream_port}: {exc}")

        self.create_timer(5.0, self._heartbeat)
        self.get_logger().warning(
            f"Thermal SIM: {self.raw_topic} (mono16, {self.resolution} K/count) "
            f"-> thermal/image (32FC1, degrees C)"
            + (f", stream on http://localhost:{stream_port}/" if self.stream else "")
            + f". Noise {self.noise_c:.2f} C, min_contrast {self.min_contrast:.1f} C.")

    def _heartbeat(self):
        if self.frames == 0:
            self.get_logger().error(
                f"No thermal frames on {self.raw_topic} yet. Check that the "
                "ros_gz bridge has that topic, that the aircraft was spawned "
                "from x500_drone_thermal.urdf.xacro, and that the world's "
                "Sensors system asks for ogre2 (the thermal camera is not "
                "implemented in ogre1 and fails silently there).",
                throttle_duration_sec=10.0)

    def raw_callback(self, msg):
        if msg.height != H or msg.width != W:
            if not self.warned_shape:
                self.warned_shape = True
                self.get_logger().error(
                    f"{self.raw_topic} is {msg.width}x{msg.height}, expected "
                    f"{W}x{H}. thermal_cam.xacro's <image> and thermal_common's "
                    "H/W must agree; nothing downstream will resize for you.")
            return
        if msg.encoding not in ('mono16', '16UC1'):
            if not self.warned_shape:
                self.warned_shape = True
                self.get_logger().error(
                    f"{self.raw_topic} is '{msg.encoding}', expected mono16. "
                    "The gz thermal camera publishes L16; if this says rgb8 "
                    "the bridge is pointed at the wrong sensor.")
            return

        dtype = np.dtype(np.uint16).newbyteorder('>' if msg.is_bigendian else '<')
        counts = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(H, W)
        grid = counts.astype(np.float32) * self.resolution - 273.15
        if self.noise_c > 0.0:
            grid = grid + self._rng.normal(0.0, self.noise_c, grid.shape).astype(np.float32)
        self.frames += 1

        # WALL clock, not the sim clock the bridge stamped this with. See
        # "WHY THE TIMESTAMP IS REWRITTEN" in the header.
        now = time.time()
        stamp = Time()
        stamp.sec = int(now)
        stamp.nanosec = int((now - int(now)) * 1e9)

        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = 'thermal_camera'
        out.height, out.width = H, W
        out.encoding = '32FC1'
        out.is_bigendian = 0
        out.step = W * 4
        out.data = grid.tobytes()
        self.image_pub.publish(out)

        blobs, ambient = find_blobs(grid.astype(float), self.min_contrast,
                                    min_pixels=self.min_blob_pixels)
        r, c = np.unravel_index(int(np.argmax(grid)), grid.shape)
        self.hotspot_pub.publish(String(
            data=f"{float(grid[r, c]):.2f}|{int(r)}|{int(c)}|{ambient:.2f}"))

        if self.preview_pub is not None or self.stream is not None:
            self._render(grid.astype(float), blobs, ambient, stamp)

    def _render(self, grid, blobs, ambient, stamp):
        import cv2
        img = annotate(grid, blobs, ambient, scale=self.preview_scale,
                       note=f"SIM #{self.frames}")
        if self.stream is not None:
            ok, buf = cv2.imencode('.jpg', img,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                with self._jpeg_lock:
                    self._jpeg = buf.tobytes()
        if self.preview_pub is not None:
            out = Image()
            out.header.stamp = stamp
            out.header.frame_id = 'thermal_camera'
            out.height, out.width = img.shape[0], img.shape[1]
            out.encoding = 'bgr8'
            out.step = img.shape[1] * 3
            out.data = img.tobytes()
            self.preview_pub.publish(out)

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def destroy_node(self):
        # The summary FIRST. How many frames were converted is the one thing
        # worth knowing when this node is shut down -- "0" says the bridge or
        # the render engine is wrong -- and tearing the HTTP server down
        # before saying it risks losing it to whatever the teardown does.
        self.get_logger().warning(f"Thermal SIM: {self.frames} frames converted.")
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalSim()
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
