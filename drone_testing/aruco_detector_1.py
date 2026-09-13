#!/usr/bin/env python3
"""Handheld bench test for the contrast-pad detector - camera only, no FCU.

Runs exactly the same detection the flight uses (pad_detector_node.analyse and
.judge are imported, never copied, so the two cannot drift apart) but with no
flight dependencies at all. Carry the drone, or just the camera, hold it at a
series of heights over the pad and watch what the algorithm actually sees.

    ros2 run lend aruco_detector
    # then open http://<jetson>:8081/ on your laptop

The preview shows the decision circle, the DESCEND / reason text, and a live
readout of coverage, contrast, black/white split and void fraction - the same
numbers that gate the real descent.

Height is optional: if the uXRCE-DDS agent happens to be running it is taken
from the rangefinder, otherwise every row simply reports height unknown. Hold
the camera at a measured height and pass it with `-p assumed_height:=1.2` to
tag the log instead.

Stereo note: the ZED enumerates as a single 1344x376 side-by-side pair. Feeding
that whole frame to the detector would put the seam and the right lens inside
the analysis circle, so `stereo_half` crops to one lens first.

Despite the filename this is no longer an ArUco node - the landing system is
contrast-based. The name is kept so existing launch files and habits still work.
"""

import csv
import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point

from lend.pad_detector_node import PadDetectorNode, MjpegServer, open_camera


class PadBenchNode(PadDetectorNode):
    """PadDetectorNode with the flight plumbing swapped for bench reporting.

    Subclassing keeps one implementation of the algorithm: analyse(), judge(),
    colour_check() and the rest are inherited verbatim. Only the camera source,
    the logging and the overlay differ.
    """

    CSV_COLUMNS = [
        't', 'frame', 'ready', 'status',
        'height', 'height_src',
        'coverage', 'contrast', 'black_frac', 'white_frac',
        'void_frac', 'colour_frac',
        'offset_x', 'offset_y', 'offset_norm',
        'mean_level', 'spread', 'otsu',
    ]

    def __init__(self):
        # Skip PadDetectorNode.__init__ - it wires flight publishers and opens
        # the camera before we can apply the bench-specific device defaults.
        Node.__init__(self, 'pad_bench_node')

        # Default to the mono cam by identity, not by number: /dev/videoN moves
        # between boots and video0 has been the ZED for most of this project.
        self.declare_parameter(
            'video_device', '/dev/v4l/by-id/usb-046d_0823_1469ADD0-video-index0')
        self.declare_parameter('fallback_device', '/dev/video0')
        # 'none', 'left' or 'right' - crop a side-by-side stereo frame.
        self.declare_parameter('stereo_half', 'auto')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('fourcc', 'YUYV')
        self.declare_parameter('flip_180', False)

        # Detection parameters - same names and defaults as the flight node, so
        # whatever you tune here can be passed straight to the real thing.
        self.declare_parameter('circle_radius', 0.40)
        self.declare_parameter('coverage_threshold', 0.80)
        self.declare_parameter('min_black_frac', 0.15)
        self.declare_parameter('min_white_frac', 0.15)
        self.declare_parameter('min_contrast', 40.0)
        self.declare_parameter('hold_frames', 5)
        self.declare_parameter('close_height', 0.50)
        self.declare_parameter('cutoff_height', 0.25)
        self.declare_parameter('cutoff_arm_height', 0.80)
        self.declare_parameter('descent_rate', 0.10)
        self.declare_parameter('require_height', False)   # bench runs without an FCU
        self.declare_parameter('pad_hue', 67)
        self.declare_parameter('pad_hue_tolerance', 25)
        self.declare_parameter('hue_ignore_below_sat', 15)
        self.declare_parameter('colour_saturation', 60)
        self.declare_parameter('max_colour_frac', 0.15)
        self.declare_parameter('colour_min_value', 60)
        self.declare_parameter('nudge_step', 0.055)
        self.declare_parameter('void_deadband', 0.08)
        self.declare_parameter('tone_band_frac', 0.30)
        self.declare_parameter('two_tone_memory', 4.0)
        self.declare_parameter('uniform_spread', 30.0)
        self.declare_parameter('black_level', 90)
        self.declare_parameter('white_level', 200)
        self.declare_parameter('tone_priority_height', 0.50)

        # Port 8081 so this can run alongside a live pad_detector_node on 8080.
        self.declare_parameter('stream_port', 8081)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('stream_every', 2)
        self.declare_parameter('reopen_after', 45)
        self.declare_parameter('log_csv', True)
        self.declare_parameter('log_dir', '/home/ark-jetson-orin/Downloads/cam/lend/logs')
        self.declare_parameter('report_period', 2.0)
        # Tag the log with a height you measured by hand, when there is no FCU.
        self.declare_parameter('assumed_height', -1.0)

        g = self.get_parameter
        self.device = g('video_device').value
        self.fallback_device = g('fallback_device').value
        self.stereo_half = str(g('stereo_half').value)
        self.width = int(g('width').value)
        self.height = int(g('height').value)
        self.fps = float(g('fps').value)
        self.fourcc = g('fourcc').value
        self.flip_180 = g('flip_180').value
        self.circle_radius = float(g('circle_radius').value)
        self.coverage_threshold = float(g('coverage_threshold').value)
        self.min_black_frac = float(g('min_black_frac').value)
        self.min_white_frac = float(g('min_white_frac').value)
        self.min_contrast = float(g('min_contrast').value)
        self.hold_frames = int(g('hold_frames').value)
        self.close_height = float(g('close_height').value)
        self.cutoff_height = float(g('cutoff_height').value)
        self.cutoff_arm_height = float(g('cutoff_arm_height').value)
        self.max_height_seen = 0.0
        self.descent_rate = float(g('descent_rate').value)
        self.require_height = bool(g('require_height').value)
        self.pad_hue = int(g('pad_hue').value)
        self.pad_hue_tolerance = int(g('pad_hue_tolerance').value)
        self.hue_ignore_below_sat = int(g('hue_ignore_below_sat').value)
        self.colour_saturation = int(g('colour_saturation').value)
        self.max_colour_frac = float(g('max_colour_frac').value)
        self.colour_min_value = int(g('colour_min_value').value)
        self.nudge_step = float(g('nudge_step').value)
        self.void_deadband = float(g('void_deadband').value)
        self.tone_band_frac = float(g('tone_band_frac').value)
        self.two_tone_memory = float(g('two_tone_memory').value)
        self.last_two_tone = 0.0
        self.uniform_spread = float(g('uniform_spread').value)
        self.black_level = float(g('black_level').value)
        self.white_level = float(g('white_level').value)
        self.tone_priority_height = float(g('tone_priority_height').value)
        self.stream_port = int(g('stream_port').value)
        self.jpeg_quality = int(g('jpeg_quality').value)
        self.stream_every = max(1, int(g('stream_every').value))
        self.reopen_after = int(g('reopen_after').value)
        self.log_csv = bool(g('log_csv').value)
        self.log_dir = os.path.expanduser(str(g('log_dir').value))
        self.report_period = float(g('report_period').value)
        assumed = float(g('assumed_height').value)
        self.assumed_height = None if assumed < 0 else assumed

        # Bench topics, so this never collides with a running flight detector.
        self.ready_pub = self.create_publisher(Bool, '/bench/ready', 10)
        self.status_pub = self.create_publisher(String, '/bench/status', 10)
        self.offset_pub = self.create_publisher(Point, '/bench/offset', 10)
        self.nudge_pub = self.create_publisher(Point, '/bench/nudge', 10)
        self.height_pub = self.create_publisher(Float32, '/bench/height', 10)

        # No FCU on the bench: height stays unknown unless assumed_height is set.
        self.range_m = None
        self.range_time = 0.0
        self.dist_bottom = None
        self.ekf_height = None

        self.frames_total = 0
        self.frames_ready = 0
        self.good_run = 0
        self.longest_run = 0
        self.ready = False
        self.started = self.now()
        self.last_report = self.now()
        self.last_status = None
        self.mask_cache = None
        self.last_shape = (self.height, self.width)
        self.phase = 'FAR'
        self.last_height = None
        self.last_height_src = 'none'
        self.last_descent = 0.0
        self.rejected = None

        self.stream = None
        if self.stream_port > 0:
            try:
                self.stream = MjpegServer(self.stream_port, self.get_logger())
            except OSError as exc:
                self.get_logger().error(f'no MJPEG stream: {exc}. Continuing.')

        self.csv_file = self.csv_writer = self.csv_path = None
        self.open_csv()

        self.cap = None
        self.read_failures = 0
        self.last_open_warn = None
        self.opened_device = None
        if not self.try_open_camera(force_log=True) and self.fallback_device:
            self.get_logger().warn(
                f'falling back to {self.fallback_device}')
            self.device = self.fallback_device
            self.try_open_camera(force_log=True)

        # Guard: anything analyse()/judge() reads must exist on this subclass.
        required = [
            'circle_radius', 'coverage_threshold', 'min_black_frac',
            'min_white_frac', 'min_contrast', 'close_height', 'cutoff_height',
            'cutoff_arm_height',
            'descent_rate', 'pad_hue', 'pad_hue_tolerance', 'hue_ignore_below_sat',
            'colour_saturation',
            'max_colour_frac', 'colour_min_value', 'nudge_step', 'void_deadband', 'tone_band_frac',
            'two_tone_memory', 'uniform_spread',
            'black_level', 'white_level', 'tone_priority_height',
        ]
        missing = [a for a in required if not hasattr(self, a)]
        if missing:
            raise RuntimeError(
                'pad bench is missing parameters the shared detector needs: '
                + ', '.join(missing)
                + ' - add them alongside the others in PadBenchNode.__init__')

        self.get_logger().info(
            f'pad bench up: circle r={self.circle_radius:.2f}, '
            f'need {self.coverage_threshold * 100:.0f}% coverage. '
            f'Preview on port {self.stream_port}. Hold it over the pad and '
            f'change height slowly.')

        self.create_timer(1.0 / self.fps, self.tick)

    # ---------- height: bench sources only ----------

    def fused_height(self):
        """Bench height: whatever you told us, else unknown.

        Deliberately does not consult the FCU - this node is meant to run with
        nothing but a camera plugged in.
        """
        if self.assumed_height is not None:
            return self.assumed_height, 'assumed'
        return None, 'none'

    # ---------- camera ----------

    def split_stereo(self, frame):
        """Crop a side-by-side stereo frame to one lens.

        The ZED presents both lenses in a single wide frame (1344x376). Left
        untouched, the seam and the second lens land inside the analysis circle
        and every metric is measured on the wrong thing.
        """
        if frame is None:
            return frame
        h, w = frame.shape[:2]
        half = self.stereo_half
        if half == 'auto':
            half = 'left' if w >= 2 * h else 'none'
        if half == 'left':
            return frame[:, : w // 2]
        if half == 'right':
            return frame[:, w // 2:]
        return frame

    # ---------- CSV ----------

    def open_csv(self):
        if not self.log_csv:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_path = os.path.join(self.log_dir, f'bench_{stamp}.csv')
            self.csv_file = open(self.csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.CSV_COLUMNS)
            self.get_logger().info(f'logging to {self.csv_path}')
        except OSError as exc:
            self.get_logger().error(f'could not open log: {exc}. Continuing.')
            self.csv_file = self.csv_writer = None

    # ---------- main loop ----------

    def tick(self):
        if self.cap is None or not self.cap.isOpened():
            self.try_open_camera()
            return
        ok, frame = self.cap.read()
        if not ok:
            self.read_failures += 1
            if self.read_failures % 30 == 1:
                self.get_logger().warn(f'camera read failed ({self.read_failures}x)')
            if self.read_failures >= self.reopen_after:
                self.get_logger().error(
                    f'{self.read_failures} failed reads - reopening {self.device}')
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None
                self.read_failures = 0
                self.try_open_camera(force_log=True)
            return
        self.read_failures = 0

        frame = self.split_stereo(frame)
        if self.flip_180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frames_total += 1

        height, height_src = self.fused_height()
        m = self.analyse(gray, frame)

        if (m is not None and m['contrast'] >= self.min_contrast
                and m['black_frac'] >= self.min_black_frac
                and m['white_frac'] >= self.min_white_frac):
            self.last_two_tone = self.now()

        passed, status, phase, descent = self.judge(m, height)

        if m is not None and phase != 'CUTOFF' and m['void_frac'] > self.void_deadband:
            _, radius_px = self.circle_mask(gray.shape)
            gain = self.nudge_step / max(radius_px, 1)
            m['nudge_y'] = float(m['offset_x']) * gain
            m['nudge_x'] = -float(m['offset_y']) * gain

        if passed:
            self.good_run += 1
            self.longest_run = max(self.longest_run, self.good_run)
        else:
            self.good_run = 0
        self.ready = self.good_run >= self.hold_frames
        if passed and not self.ready:
            status = f'HOLDING ({self.good_run}/{self.hold_frames})'
        if self.ready:
            self.frames_ready += 1

        self.phase = phase
        self.last_height = height
        self.last_height_src = height_src
        self.last_descent = descent if self.ready else 0.0

        self.ready_pub.publish(Bool(data=self.ready))
        self.status_pub.publish(String(data=status))
        if m is not None:
            self.offset_pub.publish(Point(
                x=float(m['offset_x']), y=float(m['offset_y']),
                z=float(m['coverage'])))
            self.nudge_pub.publish(Point(
                x=float(m['nudge_x']), y=float(m['nudge_y']),
                z=float(self.last_descent)))

        if status != self.last_status:
            self.get_logger().info(
                f'{status}'
                + ('' if m is None else
                   f"  cov={m['coverage'] * 100:.0f}% contrast={m['contrast']:.0f} "
                   f"void={m['void_frac'] * 100:.0f}% "
                   f"b/w={m['black_frac'] * 100:.0f}/{m['white_frac'] * 100:.0f}"))
            self.last_status = status

        if self.csv_writer is not None and m is not None:
            self.csv_writer.writerow([
                f'{self.now() - self.started:.3f}', self.frames_total,
                int(self.ready), status,
                '' if height is None else f'{height:.3f}', height_src,
                f"{m['coverage']:.4f}", f"{m['contrast']:.1f}",
                f"{m['black_frac']:.4f}", f"{m['white_frac']:.4f}",
                f"{m['void_frac']:.4f}", f"{m['colour_frac']:.4f}",
                f"{m['offset_x']:.1f}", f"{m['offset_y']:.1f}",
                f"{m['offset_norm']:.3f}",
                f"{m['mean_level']:.1f}", f"{m['spread']:.1f}", f"{m['otsu']:.0f}",
            ])

        self.report()
        self.draw(frame, m, status)

    def report(self):
        if self.now() - self.last_report < self.report_period:
            return
        self.last_report = self.now()
        if self.frames_total == 0:
            return
        pct = 100.0 * self.frames_ready / self.frames_total
        self.get_logger().info(
            f'{self.frames_ready}/{self.frames_total} frames would descend '
            f'({pct:.0f}%)  longest run {self.longest_run}')

    def destroy_node(self):
        if self.frames_total:
            pct = 100.0 * self.frames_ready / self.frames_total
            self.get_logger().info(
                f'FINAL: {self.frames_ready}/{self.frames_total} frames ready '
                f'({pct:.0f}%), longest continuous run {self.longest_run} frames')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PadBenchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
