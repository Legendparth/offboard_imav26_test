"""
Find the VERTICAL tubes of the tube obstacle and publish where each one is, in
angles and depth.

This is tube_cross.py's eyes, and it is the same shape of node as bar_detect:
HSV mask -> shape test -> depth samples -> /tube_geometry, with a debounced
/tubes_detected for the flight node to gate on. Topics, intrinsics, the MJPEG
stream and the frame-rate cap are wired exactly as in window_detect and
bar_detect.

THE OBSTACLE
    Three uprights 0.5 m apart and 2 m tall, a horizontal tube joining them at
    0.461 m, and a diagonal from the top of the left upright down to the right
    one. Behind it, about 1 m back, one more free-standing upright.

    Only the UPRIGHTS are measured. They are what the flight node needs -- the
    ground position of each one fixes the plane of the obstacle and the gap
    between the left and middle tubes -- and they are the only part whose
    position does not depend on which height the camera happens to be at.

WHY THE MASK IS OPENED WITH A TALL KERNEL
    The whole obstacle is one colour and one connected piece: the uprights,
    the cross tube and the diagonal are a single contour in a colour mask, and
    no shape test on that contour finds three tubes in it.

    A morphological OPEN with a kernel one pixel wide and vertical_kernel_frac
    of the image tall keeps only what has a long vertical run in every column
    it occupies. An upright has that. The cross tube has a run of its own
    thickness. The diagonal, at 45 degrees, has a run of its thickness times
    root two. Both vanish, and what is left falls apart into one contour per
    upright.

THE FLOOR
    The arena floor is red too. A floor strip that survives the open is wedge-
    shaped and its depth changes along it by metres, so it fails the depth
    spread test here. Anything that gets past that is placed in NED by the
    flight node and has to stand on the floor and reach up past
    min_tube_top_height, which the floor cannot.

THE HOLE
    The uprights alone do not say where to fly. The diagonal cuts the front
    face into two cells, and which of them is the big one depends on which way
    round the obstacle is built and which way the aircraft came at it -- the
    template cannot know that, and getting it backwards is how you end up
    threading the small cell right under the diagonal.

    So the hole is measured too. The colour mask is closed, and the contours
    are taken with RETR_CCOMP: the front structure is one outer contour and
    every fully enclosed cell in it is a child of that contour. The
    biggest of those children IS the trapezium under the diagonal, whatever
    the handedness, and its centre of area is where the aircraft should go.
    The children are published on /tube_hole at the depth of the uprights,
    biggest first, and the flight node aims at the first one that stands at a
    plausible height off the floor -- the cells UNDER the cross tube are
    enclosed too, and being nearer the camera they can look bigger.

    The edges are measured through the centroid, not around the bounding
    box, and that matters: the roof of the big cell is the sloping diagonal,
    so the top of the bounding box is the high CORNER of the cell while what
    the aircraft has to fit under is the diagonal directly over its own track.
    Measuring it there is also what stops a wrong-handed template from flying
    the aircraft into the diagonal -- the camera sees which way the thing
    really slopes.

    A cell that runs off the edge of the frame is not enclosed and so is not
    found at all -- which is the right answer, since its centroid would be the
    centroid of whatever part happened to be in frame.

TOPICS
    /tubes_detected     Bool, debounced: at least min_tubes uprights in frame
    /tube_info          String, one line of human-readable state
    /tube_geometry      Float32MultiArray, one row of STRIDE values per tube:
                            depth_m      median depth along the centreline
                            az_deg       azimuth of the centreline, + RIGHT
                            el_deg       elevation of the depth sample, + UP
                            el_top_deg   elevation of the visible top end
                            el_bot_deg   elevation of the visible bottom end
                            trunc_top    1.0 if the top is cut by the frame
                            trunc_bot    1.0 if the bottom is cut by the frame
                            width_px     apparent thickness
                        An EMPTY array means "frame processed, no tubes".
    /tube_hole          Float32MultiArray, the enclosed holes of the front
                        structure, biggest first, HOLE_STRIDE values each:
                            az_deg       azimuth of the centre of area, + RIGHT
                            el_deg       elevation of the centre of area, + UP
                            az_left_deg  azimuth of its left edge, ACROSS
                            az_right_deg ... and its right edge, ACROSS the
                                         centre of area -- not of the bounding
                                         box
                            el_top_deg   elevation of the edge DIRECTLY ABOVE
                            el_bot_deg   ... and DIRECTLY BELOW it
                            el_top_l_deg elevation of the roof a quarter of
                            el_top_r_deg the way in from each side. Their
                                         difference is which way the roof
                                         SLOPES -- the one thing the template
                                         cannot know, and the thing that
                                         decides which way it is safe to lean.
                            depth_m      median depth of the uprights around it
                            area_px      its area in the image
                            sides        4.0 if it is a clean quadrilateral
                                         (window_detect's approx_quad agrees
                                         with the contour), else 0.0
                            roof_cut     1.0 if the cell's roof is above the
                                         top of the frame, so el_top_deg (and
                                         the two shoulders) are the edge of
                                         the image and not the diagonal. The
                                         cell is then TALLER than measured,
                                         never shorter, so every height that
                                         comes off it is a safe lower bound.
                        An EMPTY array means "no enclosed hole this frame".
                        Biggest in the IMAGE is not always biggest on the
                        obstacle -- see _find_holes -- so the flight node
                        takes the first plausible one, preferring a quad.
    /tube_detection/image[/compressed]   the debug overlay

    q/k mean nothing here; this node does not fly anything.
"""

import math
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, MultiArrayDimension, String

from drone_testing.bar_detect import rect_axes
from drone_testing.window_detect import (HSV_RANGES, MjpegServer, approx_quad,
                                         array_to_imgmsg,
                                         get_median_depth, hsv_mask,
                                         imgmsg_to_bgr, imgmsg_to_depth)

STRIDE = 8
HOLE_STRIDE = 12


def vertical_tubes(mask, kernel_px, min_area, min_aspect, max_tilt_deg):
    """Every upright-shaped contour left after a vertical open, left to right.

    Returns (opened_mask, [(contour, centre, half, long_len, short_len,
    aspect, tilt), ...]).
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, max(3, int(kernel_px))))
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        centre, half, long_len, short_len = rect_axes(cv2.minAreaRect(contour))
        if short_len < 1.0:
            continue
        aspect = long_len / short_len
        if aspect < min_aspect:
            continue
        # Angle of the long axis off VERTICAL in the image, folded to [0, 90].
        tilt = abs(math.degrees(math.atan2(half[0], half[1])))
        if tilt > 90.0:
            tilt = 180.0 - tilt
        if tilt > max_tilt_deg:
            continue
        found.append((contour, centre, half, long_len, short_len, aspect, tilt))
    found.sort(key=lambda f: f[1][0])
    return opened, found


def quad_hole(contour, min_solidity, max_area_error):
    """The cell as a four-sided polygon, or None if it is not cleanly one.

    Same idea as the window: hull, then approxPolyDP down a ladder of
    epsilons until it comes out with four vertices (approx_quad is
    window_detect's, unchanged). Two gates on top of it, because unlike the
    window this contour is a HOLE and a hole can be any shape:

      * solidity, the contour's own area over its hull's. The cell of the
        obstacle is a trapezium and fills its hull. A cell with the back
        upright poking into it, or two cells joined through a gap in the
        mask, does not.
      * the quad's area against the contour's, which catches a four-vertex
        fit that has cut a corner off.

    Where it holds, the quad is the better shape to measure: a nick in the
    mask along one side moves the contour's centre of area and shortens the
    run measured through it, and does neither to the polygon fitted over it.
    """
    area = cv2.contourArea(contour)
    if area <= 0.0:
        return None
    hull = cv2.convexHull(contour)
    hull_area = cv2.contourArea(hull)
    if hull_area <= 0.0 or area / hull_area < min_solidity:
        return None
    quad = approx_quad(hull)
    if quad is None or not cv2.isContourConvex(quad):
        return None
    quad_area = cv2.contourArea(quad)
    if quad_area <= 0.0 or abs(quad_area - area) / area > max_area_error:
        return None
    return quad


def run_through(binary, u, v, vertical):
    """The extent of the run of set pixels through (u, v), or None.

    Used to measure a cell where the aircraft will actually fly through it --
    the column and the row through its centre of area.
    """
    line = binary[:, u] if vertical else binary[v, :]
    i = v if vertical else u
    if i < 0 or i >= line.size or not line[i]:
        return None
    lo = hi = i
    while lo > 0 and line[lo - 1]:
        lo -= 1
    while hi < line.size - 1 and line[hi + 1]:
        hi += 1
    return lo, hi


def enclosed_holes(mask, close_px, min_area, seal_top=True):
    """Every fully enclosed hole in the colour mask, biggest first.

    Each is (centre, (x, y, w, h), area, contour, roof_cut). RETR_CCOMP gives
    the structure as an outer contour with one child per enclosed cell; the
    close first joins the tubes at the welds so the cells really are enclosed.

    seal_top lays one set row across the TOP of the image before the contours
    are found. Without it the TALL cell of a diagonal-braced gate is not a
    cell at all: its roof is the high end of the diagonal, that end is above
    the top of the frame from anywhere close enough to measure the obstacle,
    and a region open at the top is not a child contour and never becomes a
    candidate. The short cell under the low end always is, so the detector
    silently offers only the small opening. One row closes exactly those
    cells and nothing else -- the arena behind the structure is still open at
    the sides and the bottom, so it stays unenclosed -- and what comes back
    carries roof_cut so the caller knows the ceiling it measures is the edge
    of the image rather than the diagonal.
    """
    k = max(3, int(close_px))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    pad = 1 if seal_top else 0
    if pad:
        closed = cv2.copyMakeBorder(closed, pad, 0, 0, 0,
                                    cv2.BORDER_CONSTANT, value=255)
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_CCOMP,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    height = mask.shape[0]
    found = []
    for contour, node in zip(contours, hierarchy[0]):
        if node[3] < 0:                     # an outer boundary, not a hole
            continue
        if pad:
            # Back into image coordinates, with the sealing row folded onto
            # row 0 so a cut roof reads as "the cell reaches the top".
            contour = contour.copy()
            contour[:, :, 1] = np.clip(contour[:, :, 1] - pad, 0, height - 1)
        area = cv2.contourArea(contour)
        if area < min_area:
            continue
        m = cv2.moments(contour)
        if m['m00'] <= 0.0:
            continue
        centre = (m['m10'] / m['m00'], m['m01'] / m['m00'])
        rect = cv2.boundingRect(contour)
        found.append((centre, rect, area, contour, rect[1] <= 0))
    found.sort(key=lambda f: -f[2])
    return found


class TubeDetect(Node):

    IMAGE_TOPIC = '/zed/zed_node/rgb/color/rect/image'
    DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered'
    CAMERA_INFO_TOPIC = 'auto'

    FALLBACK_HFOV_DEG = 90.0
    MAX_FPS = 10.0

    DETECT_FRAMES = 3
    LOST_FRAMES = 5
    MIN_TUBES = 2               # uprights in frame for /tubes_detected

    MIN_AREA = 600
    MIN_ASPECT = 4.0
    MAX_TILT_DEG = 15.0         # of the long axis off vertical, in the image
    VERTICAL_KERNEL_FRAC = 0.10 # of the image height. Must be longer than the
                                # vertical run of the diagonal where it is
                                # thickest in the image (up close), and shorter
                                # than the visible length of an upright.
    HOLE_CLOSE_FRAC = 0.02      # of the image height. Closes the welds so a
                                # cell counts as enclosed; must stay well under
                                # the smallest cell.
    HOLE_MIN_AREA_FRAC = 0.01   # of the image area
    HOLE_MIN_SOLIDITY = 0.80    # contour area / hull area before a quad is
                                # fitted to it at all
    HOLE_MAX_AREA_ERROR = 0.20  # how far the fitted quad may be off the
                                # contour's own area
    MAX_HOLES = 5               # published per frame, biggest first. Sealing
                                # the top of the frame makes the space ABOVE
                                # the diagonal enclosed too, and it outranks
                                # the real cell under it on pixels, so there
                                # have to be enough slots for both. Only the
                                # flight node knows how high off the floor
                                # each one is, and that is where they go.
    FAR_TUBE_BAND = 0.40        # m behind the nearest upright past which a
                                # tube is taken to be the one BEHIND the
                                # obstacle and is rubbed out of the hole search
    FAR_TUBE_GROW = 3           # px the rub-out is widened by. Small on
                                # purpose: every pixel of it has to be closed
                                # back over where the far tube crossed the
                                # cross tube and the diagonal.
    SAMPLES_ALONG = 9
    SAMPLE_BOX = 2
    BORDER_MARGIN = 8
    ALLOW_ROOF_CUT = True       # keep a cell whose roof is above the frame
    DEPTH_MAX_AGE = 0.5
    DEPTH_MIN = 0.30
    DEPTH_MAX = 10.0
    LOG_PERIOD = 1.0
    JPEG_QUALITY = 60
    STREAM_PORT = 8082          # 8080 is window_detect, 8081 bar_detect

    def _num(self, name, default):
        """Numeric parameter that takes `3` and `3.0` alike (see bar_detect)."""
        from rcl_interfaces.msg import ParameterDescriptor
        return self.declare_parameter(
            name, default, ParameterDescriptor(dynamic_typing=True)).value

    def __init__(self):
        super().__init__('tube_detect')

        self.image_topic = str(self.declare_parameter('image_topic', self.IMAGE_TOPIC).value)
        self.depth_topic = str(self.declare_parameter('depth_topic', self.DEPTH_TOPIC).value)
        self.camera_info_topic = str(self.declare_parameter(
            'camera_info_topic', self.CAMERA_INFO_TOPIC).value).strip()
        if self.camera_info_topic in ('', 'auto'):
            self.camera_info_topic = self.image_topic.rsplit('/', 1)[0] + '/camera_info'
            self.get_logger().info(
                f"camera_info_topic derived from image_topic: {self.camera_info_topic}")

        self.color = str(self.declare_parameter('color', 'red').value).strip().lower()
        if self.color not in HSV_RANGES:
            raise SystemExit(
                f"Unknown color '{self.color}'; expected one of {sorted(HSV_RANGES)}")

        self.min_area = float(self._num('min_area', float(self.MIN_AREA)))
        self.min_aspect = float(self._num('min_aspect', self.MIN_ASPECT))
        self.max_tilt_deg = float(self._num('max_tilt_deg', self.MAX_TILT_DEG))
        self.kernel_frac = float(self._num('vertical_kernel_frac', self.VERTICAL_KERNEL_FRAC))
        self.far_tube_band = float(self._num('far_tube_band', self.FAR_TUBE_BAND))
        self.far_tube_grow = float(self._num('far_tube_grow', float(self.FAR_TUBE_GROW)))
        self.hole_close_frac = float(self._num('hole_close_frac', self.HOLE_CLOSE_FRAC))
        self.hole_min_area_frac = float(self._num('hole_min_area_frac',
                                                  self.HOLE_MIN_AREA_FRAC))
        self.hole_min_solidity = float(self._num('hole_min_solidity',
                                                 self.HOLE_MIN_SOLIDITY))
        self.hole_max_area_error = float(self._num('hole_max_area_error',
                                                   self.HOLE_MAX_AREA_ERROR))
        self.samples_along = int(self._num('samples_along', self.SAMPLES_ALONG))
        self.border_margin = float(self._num('border_margin', float(self.BORDER_MARGIN)))
        self.allow_roof_cut = bool(self._num('allow_roof_cut', self.ALLOW_ROOF_CUT))
        self.min_tubes = int(self._num('min_tubes', self.MIN_TUBES))
        self.detect_frames = int(self._num('detect_frames', self.DETECT_FRAMES))
        self.lost_frames = int(self._num('lost_frames', self.LOST_FRAMES))
        self.depth_min = float(self._num('depth_min', self.DEPTH_MIN))
        self.depth_max = float(self._num('depth_max', self.DEPTH_MAX))
        self.fallback_hfov = math.radians(float(self._num(
            'fallback_hfov_deg', self.FALLBACK_HFOV_DEG)))
        self.max_fps = float(self._num('max_fps', self.MAX_FPS))
        self.min_frame_interval = (1.0 / self.max_fps) if self.max_fps > 0.0 else 0.0

        self.publish_image = bool(self.declare_parameter('publish_image', True).value)
        self.publish_compressed = bool(self.declare_parameter('publish_compressed', True).value)
        self.publish_mask = bool(self.declare_parameter('publish_mask', False).value)
        self.jpeg_quality = int(self._num('jpeg_quality', self.JPEG_QUALITY))
        self.stream_port = int(self._num('stream_port', self.STREAM_PORT))
        self.stream_scale = float(self._num('stream_scale', 0.5))

        self.create_subscription(Image, self.image_topic,
                                 self.image_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic,
                                 self.depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self.camera_info_callback, qos_profile_sensor_data)

        self.detected_pub = self.create_publisher(Bool, 'tubes_detected', 10)
        self.info_pub = self.create_publisher(String, 'tube_info', 10)
        self.geometry_pub = self.create_publisher(Float32MultiArray, 'tube_geometry', 10)
        self.hole_pub = self.create_publisher(Float32MultiArray, 'tube_hole', 10)
        self.image_pub = (self.create_publisher(Image, 'tube_detection/image', 1)
                          if self.publish_image else None)
        self.mask_pub = (self.create_publisher(Image, 'tube_detection/mask', 1)
                         if self.publish_mask else None)
        self.compressed_pub = (self.create_publisher(
            CompressedImage, 'tube_detection/image/compressed', 1)
            if self.publish_compressed else None)

        self._jpeg = None
        self._jpeg_lock = threading.Lock()
        self.stream = MjpegServer(self, self.stream_port) if self.stream_port else None

        # The enable gate. A detector that is not being flown on is pure heat
        # -- an HSV convert and a contour pass per frame for a stage that
        # ended minutes ago -- and worse than the heat is the CROSS-TALK: the
        # exit wall carries a RED window as well as a blue one, and a red
        # window is a rectangle of exactly the colour the tube detector hunts
        # for. course_fsm publishes to enable_topic as it enters and leaves
        # each phase. start_enabled is what holds before anything has been
        # published, so the standalone missions -- which never publish to it
        # -- are unaffected.
        self.enabled = bool(self.declare_parameter('start_enabled', True).value)
        self.enable_topic = str(self.declare_parameter(
            'enable_topic', '~/enable').value)
        self.create_subscription(Bool, self.enable_topic, self.enable_callback, 10)

        self.depth_image = None
        self.depth_time = 0.0
        self.intrinsics = None
        self.last_processed = 0.0
        self.frames = 0
        self.frames_skipped = 0
        self.first_frame_logged = False

        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False
        self.last_log = 0.0
        self.last_info = ''

        self.get_logger().warning(
            f"Tube detection up. colour={self.color} image={self.image_topic} "
            f"depth={self.depth_topic} -> /tubes_detected, /tube_geometry"
            + (f", MJPEG on :{self.stream_port}" if self.stream else ""))

    # ------------------------------------------------------------------ subs

    def enable_callback(self, msg):
        want = bool(msg.data)
        if want != self.enabled:
            self.get_logger().warning(
                f"Detection {'ENABLED' if want else 'DISABLED'} by "
                f"{self.enable_topic}.")
            if not want:
                self._on_disable()
        self.enabled = want

    def _on_disable(self):
        """Drop the debounce so nothing stale is believed when it comes back."""
        self.hit_streak = 0
        self.miss_streak = 0
        self.detected = False
        self._publish_state('detection disabled')

    def camera_info_callback(self, msg):
        k = msg.k
        if k[0] > 0.0 and k[4] > 0.0 and self.intrinsics is None:
            self.intrinsics = (float(k[0]), float(k[4]), float(k[2]), float(k[5]))
            self.get_logger().info(
                f"CameraInfo from {self.camera_info_topic}: fx={k[0]:.1f} "
                f"fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}.")

    def depth_callback(self, msg):
        try:
            self.depth_image = imgmsg_to_depth(msg)
            self.depth_time = time.monotonic()
        except Exception as exc:
            self.get_logger().error(f"Cannot convert depth frame: {exc}",
                                    throttle_duration_sec=5.0)

    def current_depth(self):
        if self.depth_image is None:
            return None
        if time.monotonic() - self.depth_time > self.DEPTH_MAX_AGE:
            return None
        return self.depth_image

    def _intrinsics(self, shape):
        if self.intrinsics is not None:
            return self.intrinsics
        h, w = shape[:2]
        fx = (w / 2.0) / math.tan(self.fallback_hfov / 2.0)
        self.get_logger().warning(
            f"No CameraInfo on {self.camera_info_topic} yet; guessing the "
            f"intrinsics from fallback_hfov_deg={math.degrees(self.fallback_hfov):.0f}. "
            "Every tube position is scaled wrong until this arrives.",
            throttle_duration_sec=5.0)
        return fx, fx, w / 2.0, h / 2.0

    def image_callback(self, msg):
        now = time.monotonic()
        if not self.enabled:
            self.frames_skipped += 1
            return
        if self.min_frame_interval and now - self.last_processed < self.min_frame_interval:
            self.frames_skipped += 1
            return
        self.last_processed = now
        try:
            cv_image = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().error(f"Cannot convert image frame: {exc}",
                                    throttle_duration_sec=5.0)
            return
        self.frames += 1
        if not self.first_frame_logged:
            self.first_frame_logged = True
            h, w = cv_image.shape[:2]
            self.get_logger().info(f"First frame from {self.image_topic}: {w}x{h}.")
        self.process(cv_image, msg.header)

    # ------------------------------------------------------------ detection

    def process(self, cv_image, header):
        h, w = cv_image.shape[:2]
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = hsv_mask(hsv, self.color)
        opened, candidates = vertical_tubes(
            mask, self.kernel_frac * h, self.min_area, self.min_aspect, self.max_tilt_deg)

        depth_img = self.current_depth()
        rows = []
        tube_u = []
        measured = []
        rejects = []
        if depth_img is None:
            rejects.append('no depth frame')
        else:
            fx, fy, cx, cy = self._intrinsics(cv_image.shape)
            for cand in candidates:
                row, why = self._measure(cv_image, depth_img, cand, fx, fy, cx, cy)
                if row is None:
                    rejects.append(why)
                else:
                    rows.append(row)
                    tube_u.append(float(cand[1][0]))
                    measured.append((row[0], cand[0]))

        if len(rows) >= self.min_tubes:
            self._hit()
        else:
            self._miss()

        near_mask, bridge_px = self._without_far_tubes(mask, measured)
        holes, hole_why = self._find_holes(cv_image, near_mask, rows, tube_u,
                                           bridge_px)
        self._publish_geometry(rows)
        self._publish_holes(holes)
        info = (f"{len(rows)} tube(s): "
                + ", ".join(f"d={r[0]:.2f} az={r[1]:+.0f}" for r in rows))
        info += (" | holes " + ", ".join(
                     f"az={h[0]:+.0f} el={h[1]:+.0f}{' quad' if h[10] else ''}"
                     f"{' roof-cut' if h[11] else ''}"
                     for h in holes)
                 if holes else f" | no hole ({hole_why})")
        if rejects:
            info += f" | rejected: {'; '.join(rejects[:3])}"
        self.last_info = info
        self._publish_state(info)
        self._publish_images(cv_image, opened, header)
        self._log()

    def _measure(self, img, depth_img, cand, fx, fy, cx, cy):
        contour, centre, half, long_len, short_len, aspect, tilt = cand
        h, w = img.shape[:2]
        p_top = centre - half * 0.92
        p_bot = centre + half * 0.92
        if p_top[1] > p_bot[1]:
            p_top, p_bot = p_bot, p_top

        samples = []
        n = max(3, self.samples_along)
        for i in range(n):
            t = i / (n - 1.0)
            p = p_top + (p_bot - p_top) * t
            u = int(round(min(max(p[0], 0), w - 1)))
            v = int(round(min(max(p[1], 0), h - 1)))
            d = get_median_depth(depth_img, u, v, self.SAMPLE_BOX)
            if self.depth_min <= d <= self.depth_max:
                samples.append((d, (u, v)))
        if len(samples) < 3:
            return None, f'{len(samples)} depth samples'

        depths = np.array([s[0] for s in samples])
        median_depth = float(np.median(depths))
        # An upright seen from roughly level is at nearly one depth end to end.
        # A floor strip is not: that is what this catches.
        spread = float(np.max(np.abs(depths - median_depth)))
        allowed = max(0.30, 0.20 * median_depth)
        if spread > allowed:
            return None, f'depth spread {spread:.2f} m'

        # The sample nearest the median depth is the point that gets placed.
        k = int(np.argmin(np.abs(depths - median_depth)))
        su, sv = samples[k][1]

        az = math.degrees(math.atan2(float(su) - cx, fx))
        el = math.degrees(math.atan2(cy - float(sv), fy))
        el_top = math.degrees(math.atan2(cy - float(p_top[1]), fy))
        el_bot = math.degrees(math.atan2(cy - float(p_bot[1]), fy))
        trunc_top = p_top[1] < self.border_margin
        trunc_bot = p_bot[1] > h - 1 - self.border_margin

        self._draw(img, contour, p_top, p_bot, samples, median_depth, az)
        return [median_depth, az, el, el_top, el_bot,
                1.0 if trunc_top else 0.0, 1.0 if trunc_bot else 0.0,
                float(short_len)], ''

    def _without_far_tubes(self, mask, measured):
        """The mask with anything standing well behind the front plane rubbed out.

        The free-standing upright behind the obstacle is the same colour and,
        seen through the big cell, draws a line right down the middle of it --
        which splits the cell in two, and the centre of area of half a cell is
        up against a tube. It is a metre further away, so the depth says which
        contour it is, and painting it out puts the cell back together.

        Painting it out also cuts through whatever of the FRONT structure it
        crossed -- the cross tube, usually -- which would leave the cell open
        at the bottom and so not a cell at all. The width of the strip that
        was removed comes back with the mask, and the hole search closes over
        at least that much.

        Returns (mask, bridge_px).
        """
        if len(measured) < 2:
            return mask, 0.0
        near = min(d for d, _ in measured)
        far = [c for d, c in measured if d > near + self.far_tube_band]
        if not far:
            return mask, 0.0
        grow = max(3, int(self.far_tube_grow))
        out = mask.copy()
        cv2.drawContours(out, far, -1, 0, cv2.FILLED)
        cv2.drawContours(out, far, -1, 0, grow)
        # A morphological close bridges a cut of its own size only in the
        # easy direction; across a thin tube at 45 degrees it takes about
        # three times the width of the cut. Measured on the bench -- see the
        # tests in the commit that added this.
        widest = max(cv2.boundingRect(c)[2] for c in far)
        return out, 3.0 * (widest + grow) + 4.0

    def _find_holes(self, img, mask, rows, tube_u, bridge_px=0.0):
        """The enclosed cells of the structure as angles, biggest first.

        Gated on the uprights: without two of them there is no depth for the
        plane, and a centroid has to sit between the outermost uprights to be
        a cell of THIS structure rather than something else in the arena.

        More than one is published because the biggest cell in the IMAGE need
        not be the biggest cell of the obstacle -- the cells under the cross
        tube are nearer the camera and can subtend more pixels. Only the
        flight node knows how high off the floor each one is, so it does that
        rejection; this end just hands over the candidates in order.
        """
        h, w = img.shape[:2]
        if len(rows) < 2:
            return [], 'need two uprights for the plane depth'
        candidates = enclosed_holes(mask, max(self.hole_close_frac * h, bridge_px),
                                    self.hole_min_area_frac * float(h * w))
        if not candidates:
            return [], 'no enclosed cell in the mask'

        fx, fy, cx, cy = self._intrinsics(img.shape)
        depth = float(np.median([r[0] for r in rows]))
        margin = self.border_margin
        out = []
        why = 'no enclosed cell survived the gating'
        for (hu, hv), (bx, by, bw, bh), area, contour, roof_cut in candidates:
            if not (min(tube_u) <= hu <= max(tube_u)):
                why = 'centroid is outside the uprights'
                continue
            # The TOP is allowed to be cut and the other three edges are not.
            # A cell open at the side or the bottom is some other part of the
            # arena showing through; a cell open at the top is the tall cell
            # of this structure with its roof above the frame, which is the
            # one the aircraft wants and the one that used to be thrown away.
            # Its ceiling then measures as the edge of the image -- low, so
            # the crossing height that comes off it is conservative.
            if (bx <= margin or bx + bw >= w - 1 - margin
                    or by + bh >= h - 1 - margin
                    or (roof_cut and not self.allow_roof_cut)):
                why = ('cell runs off the top of the frame' if roof_cut
                       else 'cell runs off the frame')
                continue
            quad = quad_hole(contour, self.hole_min_solidity,
                             self.hole_max_area_error)
            shape = contour if quad is None else quad
            if quad is not None:
                m = cv2.moments(quad)
                hu, hv = m['m10'] / m['m00'], m['m01'] / m['m00']
            cu, cv = int(round(hu)), int(round(hv))
            filled = np.zeros((h, w), np.uint8)
            cv2.drawContours(filled, [shape], -1, 255, cv2.FILLED)
            column = run_through(filled, cu, cv, True)
            row = run_through(filled, cu, cv, False)
            if column is None or row is None:
                # A cell bent enough that its centre of area is outside it.
                why = 'centre of area is not inside the cell'
                continue
            # The roof a quarter of the way in from each side. Two numbers,
            # and their difference is the slope of the diagonal as the CAMERA
            # sees it -- no template, no handedness to get wrong.
            quarter = max(1, (row[1] - row[0]) // 4)
            shoulders = []
            for u_q in (row[0] + quarter, row[1] - quarter):
                run = run_through(filled, int(u_q), cv, True)
                shoulders.append(column[0] if run is None else run[0])
            self._draw_hole(img, shape, (cu, cv), column, row, depth,
                            len(out), quad is not None, roof_cut)
            out.append([math.degrees(math.atan2(hu - cx, fx)),
                        math.degrees(math.atan2(cy - hv, fy)),
                        math.degrees(math.atan2(float(row[0]) - cx, fx)),
                        math.degrees(math.atan2(float(row[1]) - cx, fx)),
                        math.degrees(math.atan2(cy - float(column[0]), fy)),
                        math.degrees(math.atan2(cy - float(column[1]), fy)),
                        math.degrees(math.atan2(cy - float(shoulders[0]), fy)),
                        math.degrees(math.atan2(cy - float(shoulders[1]), fy)),
                        depth, float(area), 4.0 if quad is not None else 0.0,
                        1.0 if roof_cut else 0.0])
            if len(out) >= self.MAX_HOLES:
                break
        return out, ('' if out else why)

    def _publish_holes(self, holes):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='hole', size=len(holes),
                                stride=HOLE_STRIDE * len(holes)),
            MultiArrayDimension(label='values', size=HOLE_STRIDE, stride=HOLE_STRIDE),
        ]
        msg.data = [float(v) for hole in holes for v in hole]
        self.hole_pub.publish(msg)

    def _publish_geometry(self, rows):
        msg = Float32MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label='tube', size=len(rows), stride=STRIDE * len(rows)),
            MultiArrayDimension(label='values', size=STRIDE, stride=STRIDE),
        ]
        msg.data = [float(v) for row in rows for v in row]
        self.geometry_pub.publish(msg)

    # ------------------------------------------------------------- debounce

    def _hit(self):
        self.hit_streak += 1
        self.miss_streak = 0
        if not self.detected and self.hit_streak >= self.detect_frames:
            self.detected = True
            self.get_logger().warning("TUBES DETECTED")

    def _miss(self):
        self.miss_streak += 1
        self.hit_streak = 0
        if self.detected and self.miss_streak >= self.lost_frames:
            self.detected = False
            self.get_logger().warning("Tubes LOST.")

    def _publish_state(self, info):
        msg = Bool()
        msg.data = self.detected
        self.detected_pub.publish(msg)
        text = String()
        text.data = info
        self.info_pub.publish(text)

    def _log(self):
        now = time.monotonic()
        if now - self.last_log < self.LOG_PERIOD:
            return
        self.last_log = now
        self.get_logger().info(
            f"tubes: {'YES' if self.detected else 'no '} ({self.last_info}; "
            f"{self.frames} frames, {self.frames_skipped} skipped)")

    # --------------------------------------------------------------- output

    def _draw(self, img, contour, p_top, p_bot, samples, depth, az):
        cv2.drawContours(img, [contour], -1, (255, 0, 255), 2)
        cv2.line(img, tuple(np.round(p_top).astype(int)),
                 tuple(np.round(p_bot).astype(int)), (0, 255, 0), 2)
        for _, pt in samples:
            cv2.circle(img, pt, 3, (0, 0, 255), -1)
        cv2.putText(img, f"d={depth:.2f} az={az:+.0f}",
                    (int(p_top[0]) - 40, max(20, int(p_top[1]) + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

    def _draw_hole(self, img, shape, centre, column, row, depth, rank, is_quad,
                   roof_cut=False):
        colour = (0, 255, 255) if rank == 0 else (0, 165, 255)
        cv2.drawContours(img, [shape], -1, colour, 3 if is_quad else 2)
        cu, cv_ = centre
        # The cross is what the flight node actually measures: the ceiling and
        # floor over this column, and the walls across this row.
        cv2.line(img, (cu, column[0]), (cu, column[1]), colour, 2)
        cv2.line(img, (row[0], cv_), (row[1], cv_), colour, 2)
        cv2.drawMarker(img, (cu, cv_), colour, cv2.MARKER_CROSS, 24, 2)
        cv2.putText(img, f"hole {rank}{' quad' if is_quad else ''}"
                    f"{' ROOF CUT' if roof_cut else ''} d={depth:.2f}",
                    (cu - 40, cv_ - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

    def _publish_images(self, cv_image, mask, header):
        if self.image_pub is not None:
            self.image_pub.publish(array_to_imgmsg(cv_image, 'bgr8', header))
        if self.mask_pub is not None:
            self.mask_pub.publish(array_to_imgmsg(mask, 'mono8', header))
        if self.compressed_pub is None and self.stream is None:
            return
        frame = cv_image
        if self.stream_scale and self.stream_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self.stream_scale,
                               fy=self.stream_scale, interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        payload = buf.tobytes()
        if self.compressed_pub is not None:
            out = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data = payload
            self.compressed_pub.publish(out)
        with self._jpeg_lock:
            self._jpeg = payload

    def latest_jpeg(self):
        with self._jpeg_lock:
            return self._jpeg

    def destroy_node(self):
        if self.stream is not None:
            self.stream.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TubeDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
