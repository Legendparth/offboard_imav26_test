#!/usr/bin/env python3
"""THE FLOOR LINE, from the DOWNWARD camera. Where is it, and which way does it run?

The arena floor carries a high-contrast pattern laid in a straight line from
the takeoff marker to the marker 8.7-8.8 m away. It was put there to give the
optical flow something to look at; this node uses it for the other thing a
straight line on the floor is good for, which is telling the aircraft where
the corridor is.

    down camera image -> /line/detected, /line/track

WHY THE DOWNWARD CAMERA AND NOT THE FRONT ZED

    A nadir view makes the geometry one divide. The angle of the line in the
    image IS the heading error, and the line's sideways offset from the image
    centre IS the cross-track error, in metres, as soon as you multiply by the
    rangefinder height. Nothing has to be un-projected, and nothing depends on
    the aircraft's pitch.

    From a forward-looking camera the same line is an oblique view of the
    ground plane: recovering metres needs a homography built from live pitch
    and height, and the aircraft pitches about 10 deg to hold 0.8 m/s, which
    moves the whole ground plane through the frame exactly when the control
    loop is asking where it is. The lookahead is real, but it is bought with a
    dependency on the attitude estimate at the moment it is least steady.

    The other half of the argument is the way home. A nadir detector does not
    care which way the nose points, so the aircraft can fly the return leg
    BACKWARDS -- nose still down the corridor, as thermal_fsm already does --
    instead of turning through 180 deg to put a forward camera on the line.
    That turn is not free: it spends time, it spins the airframe on an
    estimate that is only as good as the flow underneath it, and it points the
    thermal and down cameras through conventions written for the other
    heading.

WHAT IT PUBLISHES

    /line/detected      std_msgs/Bool             debounced: true only after
                                                  detect_frames consecutive
                                                  hits, false after
                                                  lost_frames misses.
    /line/track         geometry_msgs/Vector3Stamped
                            x = heading error, RADIANS. The bearing of the
                                line relative to the nose, positive when the
                                line runs off to the RIGHT, so the aircraft
                                steers to leg_bearing = heading + x.
                            y = cross-track, METRES. Signed distance of the
                                AIRCRAFT to the RIGHT of the line -- the same
                                sign convention as _follow_leg()'s `cross`, so
                                the flight node can use it without thinking.
                            z = quality, 0..1. Angular AGREEMENT (what
                                fraction of the detected line length voted for
                                the winning orientation) times SUPPORT (how
                                much line was found, against support_diag
                                image diagonals). Both, because agreement
                                alone is 1.0 whenever only two segments were
                                found and a lone mat edge would score
                                perfectly.
    /line/info          std_msgs/String           one human-readable line.
    /line/preview       sensor_msgs/Image         annotated, if preview:=true.

HOW IT FINDS THE STRIP, AND WHY NOT WITH EDGES

    The thing on the floor is not a painted line. It is a WIDE STRIP of
    patterned material -- a tropical leaf print, orange and teal and grey on
    white -- laid on dark speckled granite. That matters, because the obvious
    algorithm fails on it completely.

    Measured on a photograph of the real floor: Canny finds edges over 11% of
    the frame and HoughLinesP returns 627 segments, nearly all of them leaf
    outlines and frond veins pointing in every direction INSIDE the strip. An
    edge-and-Hough detector does not see a corridor there; it sees edge soup,
    and the longest segment in it is whichever frond happened to be crispest.

    The signal is not the pattern. It is the CONTRAST BETWEEN the strip and
    the floor: bright printed material against dark granite. So:

        blur -> Otsu threshold -> morphological close (fill the dark leaves
        so the strip becomes one solid blob) -> open (kill speckle on the
        granite) -> largest connected component -> IMAGE MOMENTS.

    The second moments of that blob give the principal axis directly -- no
    voting, no thresholds on segment length, nothing for a frond to win. On
    the real photograph this returns the same axis to within 0.5 deg across a
    9, 15 and 21 pixel morphology kernel, which is the kind of insensitivity
    to tuning that a thing in a control loop wants.

    Otsu rather than a fixed threshold because the arena's lighting is not
    ours to control and the split between "printed strip" and "granite" moves
    with it, while the fact that one is brighter than the other does not.

WHERE THE FRAMES COME FROM

    aruco_pose already owns /dev/video<n> and one v4l2 device cannot be opened
    twice, so this node does not open the camera. Run aruco_pose with
    publish_frames:=/aruco/image and point image_topic here at the same topic.
    Both detectors then see the same pixels, which is also what you want when
    you are deciding whether the marker or the line was the thing that lied.

CHECKING IT ON THE BENCH, BEFORE ANY OF IT MATTERS

    ros2 run drone_testing line_detect --ros-args \\
        -p image_topic:=/aruco/image -p stream_port:=8084

    Open http://<jetson>:8084/ , hold the aircraft over the floor pattern and
    check that walking it left makes cross-track go POSITIVE, and that yawing
    the nose right makes heading error go NEGATIVE. If either is backwards,
    fix it here with flip_lr / cam_yaw_deg -- the same dials, meaning the same
    things, as in thermal_drop.

    ros2 topic echo /line/info      the running commentary
"""

import math
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import EstimatorStatusFlags, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# These are generic image plumbing that happens to live in window_detect.py.
# Importing them beats a fourth copy, and the deliberate absence of cv_bridge
# is the whole point of them -- see the note at the top of that file: its
# compiled extension is built against the distro NumPy and a pip-installed
# NumPy 2 makes it segfault on the first frame.
from drone_testing.window_detect import (MjpegServer, array_to_imgmsg,
                                         imgmsg_to_bgr)
from drone_testing import px4_height


class LineDetect(Node):

    def __init__(self):
        super().__init__('line_detect')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.image_topic = str(p('image_topic', '/aruco/image')).strip()

        # ---- geometry. THE SAME DIALS AS THE FLIGHT NODE ----
        self.HFOV = math.radians(float(p('hfov_deg', 70.0)))
        self.CAM_YAW = math.radians(float(p('cam_yaw_deg', 0.0)))
        self.FLIP_LR = bool(p('flip_lr', False))
        self.FLIP_UD = bool(p('flip_ud', False))

        # ---- detection ----
        self.BLUR = int(p('blur_ksize', 7))
        if self.BLUR % 2 == 0:
            self.BLUR += 1              # GaussianBlur demands an odd kernel
        # CLOSE fills the dark leaves so the printed strip becomes ONE blob;
        # OPEN then removes the speckle the granite throws. Close must be big
        # enough to swallow the largest dark leaf, which is what sets it.
        self.CLOSE_K = int(p('close_ksize', 15))
        self.OPEN_K = int(p('open_ksize', 9))
        self.STRIP_IS_BRIGHT = bool(p('strip_is_bright', True))
        self.MIN_AREA_FRAC = float(p('min_area_frac', 0.05))
        self.MAX_AREA_FRAC = float(p('max_area_frac', 0.85))
        self.MIN_ELONGATION = float(p('min_elongation', 1.6))
        self.ELONGATION_REF = float(p('elongation_ref', 3.0))
        self.MIN_SOLIDITY = float(p('min_solidity', 0.60))
        self.MIN_QUALITY = float(p('min_quality', 0.35))
        self.DETECT_FRAMES = int(p('detect_frames', 2))
        self.LOST_FRAMES = int(p('lost_frames', 5))
        self.SMOOTHING = float(p('smoothing', 0.5))
        self.DOWNSCALE = float(p('downscale', 0.5))

        # ---- output ----
        self.PREVIEW = bool(p('preview', False))
        self.stream_port = int(p('stream_port', 0))
        self.JPEG_QUALITY = int(p('jpeg_quality', 70))
        self.FALLBACK_AGL = float(p('fallback_agl', 0.0))

        self.detected_pub = self.create_publisher(Bool, '/line/detected', 10)
        self.track_pub = self.create_publisher(Vector3Stamped, '/line/track', 10)
        self.info_pub = self.create_publisher(String, '/line/info', 10)
        self.preview_pub = (self.create_publisher(Image, '/line/preview', 1)
                            if self.PREVIEW else None)

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleLocalPosition,
                                 '/fmu/out/vehicle_local_position',
                                 self._on_position, px4_qos)
        self.create_subscription(EstimatorStatusFlags,
                                 px4_height.RANGEFINDER_TOPIC,
                                 self._on_flags, px4_qos)
        self.create_subscription(Image, self.image_topic, self._on_image,
                                 qos_profile_sensor_data)

        self.local_position = None
        self.estimator_flags = None
        self.hits = 0
        self.misses = 0
        self.detected = False
        self.heading_error = None       # rad, smoothed
        self.cross_track = None         # m, smoothed
        self.quality = 0.0
        self.frames = 0
        self._jpeg = None
        self._mask = None
        self._lock = threading.Lock()

        self.stream = None
        if self.stream_port:
            try:
                self.stream = MjpegServer(self, self.stream_port)
                self.get_logger().info(
                    f"Browser view on http://<jetson-ip>:{self.stream_port}/")
            except Exception as exc:
                self.get_logger().error(f"No stream on {self.stream_port}: {exc}")

        self.get_logger().warning(
            f"LINE: frames from {self.image_topic}, hfov {math.degrees(self.HFOV):.0f} "
            f"deg, cam_yaw {math.degrees(self.CAM_YAW):+.0f} deg"
            + (", flip_lr" if self.FLIP_LR else "")
            + (", flip_ud" if self.FLIP_UD else "")
            + f". Height from the rangefinder"
            + (f", falling back to {self.FALLBACK_AGL:.2f} m"
               if self.FALLBACK_AGL > 0.0 else
               " -- WITHOUT IT THE CROSS-TRACK HAS NO SCALE and only the "
               "heading error is published")
            + ". -> /line/detected, /line/track")

    # ------------------------------------------------------------- the inputs

    def _on_position(self, msg):
        self.local_position = msg

    def _on_flags(self, msg):
        self.estimator_flags = msg

    def latest_jpeg(self):
        return self._jpeg

    def _agl(self):
        """Metres to the floor, or None. fallback_agl is a BENCH crutch.

        On a bench with no PX4 at all there is no height and therefore no
        scale, so cross-track cannot be reported in metres. fallback_agl lets
        you check the geometry on a table at a known height; it is deliberately
        0.0 (off) by default, because silently inventing a height in flight
        would put a wrong-by-a-factor cross-track into the control loop.
        """
        height = px4_height.agl(self.estimator_flags, self.local_position)
        if height is not None:
            return height, True
        if self.FALLBACK_AGL > 0.0:
            return self.FALLBACK_AGL, False
        return None, False

    # ---------------------------------------------------------- the detection

    def _on_image(self, msg):
        try:
            frame = imgmsg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        self.frames += 1

        if 0.0 < self.DOWNSCALE < 1.0:
            frame = cv2.resize(frame, None, fx=self.DOWNSCALE, fy=self.DOWNSCALE,
                               interpolation=cv2.INTER_AREA)

        fit = self._find_line(frame)
        self._publish(frame, fit, msg.header)

    def _find_line(self, frame):
        """(angle_img, perp_px, quality, w, h) of the floor strip, or None.

        angle_img is the strip's principal axis in IMAGE axes, mod pi. perp_px
        is the signed perpendicular distance in pixels from the image centre to
        that axis.
        """
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (self.BLUR, self.BLUR), 0)

        # Otsu picks the split between the printed strip and the granite
        # wherever the lighting has put it. Which SIDE of the split is the
        # strip is not Otsu's to say, so it is a parameter -- bright on dark
        # for this floor.
        flag = cv2.THRESH_BINARY if self.STRIP_IS_BRIGHT else cv2.THRESH_BINARY_INV
        _, mask = cv2.threshold(blur, 0, 255, flag | cv2.THRESH_OTSU)

        if self.CLOSE_K > 1:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                          (self.CLOSE_K, self.CLOSE_K))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        if self.OPEN_K > 1:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                          (self.OPEN_K, self.OPEN_K))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        self._mask = mask

        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if count < 2:
            return None
        # Label 0 is the background. The strip is the biggest thing that is not
        # the floor -- a sunlit patch of granite is smaller and, crucially,
        # rounder, which the elongation test below throws out.
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = float(stats[best, cv2.CC_STAT_AREA])
        area_frac = area / float(w * h)
        if not (self.MIN_AREA_FRAC <= area_frac <= self.MAX_AREA_FRAC):
            return None

        comp = (labels == best).astype(np.uint8)
        m = cv2.moments(comp, binaryImage=True)
        if m['m00'] <= 0.0:
            return None
        # Normalised second central moments -> the principal axis. This is the
        # whole measurement: no voting, no segment lengths, nothing a single
        # crisp frond can win.
        mu20 = m['mu20'] / m['m00']
        mu02 = m['mu02'] / m['m00']
        mu11 = m['mu11'] / m['m00']
        angle_img = 0.5 * math.atan2(2.0 * mu11, mu20 - mu02)

        common = math.sqrt(4.0 * mu11 * mu11 + (mu20 - mu02) ** 2)
        l1 = (mu20 + mu02 + common) / 2.0
        l2 = max((mu20 + mu02 - common) / 2.0, 1e-9)
        elongation = math.sqrt(l1 / l2)
        if elongation < self.MIN_ELONGATION:
            # Round-ish. A corridor strip crossing the frame is not round, so
            # this is a mat, a puddle of light, or the pad we are sitting on.
            return None

        # Solidity: how much of its own convex hull the blob fills. A clean
        # strip is nearly solid; a blob stitched together out of unrelated
        # bright patches is not, and would otherwise pass on area alone.
        contours, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        solidity = 1.0
        if contours:
            hull = cv2.convexHull(max(contours, key=cv2.contourArea))
            hull_area = float(cv2.contourArea(hull))
            if hull_area > 0.0:
                solidity = min(1.0, area / hull_area)
        if solidity < self.MIN_SOLIDITY:
            return None

        centroid = np.array([m['m10'] / m['m00'], m['m01'] / m['m00']])
        direction = np.array([math.cos(angle_img), math.sin(angle_img)])
        normal = np.array([-direction[1], direction[0]])
        centre = np.array([w / 2.0, h / 2.0])
        perp_px = float(normal @ (centre - centroid))
        # Pin the normal to a consistent side, so the sign of perp_px means the
        # same thing frame to frame: mod pi, the axis may come back either way.
        if math.sin(angle_img) < 0.0 or (math.sin(angle_img) == 0.0
                                         and math.cos(angle_img) < 0.0):
            perp_px = -perp_px
            angle_img = math.atan2(-direction[1], -direction[0])

        # HOW ELONGATED, times HOW SOLID. Both, because a blob can be long and
        # ragged (several unrelated bright things in a row) or compact and
        # clean (the takeoff pad), and neither is a corridor.
        shape = min(1.0, (elongation - 1.0) / max(self.ELONGATION_REF - 1.0, 1e-6))
        quality = shape * solidity
        if quality < self.MIN_QUALITY:
            return None
        return angle_img, perp_px, quality, w, h

    # ----------------------------------------------------------- the geometry

    def _to_body(self, angle_img, perp_px, w, h):
        """Image line -> (heading error rad, cross-track metres or None).

        The pixel-to-angle mapping is the SAME ONE thermal_drop uses in
        pixel_ray_body(), so flip_lr / flip_ud / cam_yaw_deg mean here exactly
        what they mean there, and a dial found on the thermal bench carries
        across unchanged.
        """
        # --- direction ---
        # Image axes: +col is right across the frame, +row is DOWN the frame.
        # The same mapping as a pixel ray: right = +col, forward = -row.
        dcol, drow = math.cos(angle_img), math.sin(angle_img)
        right, forward = dcol, -drow
        if self.FLIP_LR:
            right = -right
        if self.FLIP_UD:
            forward = -forward
        # Un-rotate the camera's mounting yaw, so the answer is in AIRFRAME
        # axes rather than camera axes.
        c, s = math.cos(self.CAM_YAW), math.sin(self.CAM_YAW)
        f_body = forward * c - right * s
        r_body = forward * s + right * c
        # Mod pi again: we want the line's bearing relative to the nose in
        # (-90, +90], never "the corridor runs backwards".
        heading_error = math.atan2(r_body, f_body)
        if heading_error > math.pi / 2.0:
            heading_error -= math.pi
        elif heading_error <= -math.pi / 2.0:
            heading_error += math.pi

        # --- offset ---
        # perp_px is the offset from the image centre to the LINE. The signed
        # distance of the AIRCRAFT to the right of the line is the opposite,
        # which is the convention _follow_leg() uses for `cross`.
        height, _ = self._agl()
        if height is None:
            return heading_error, None
        # Same small-angle-free mapping as pixel_ray_body: a pixel that far
        # from centre subtends atan(frac * tan(hfov/2)).
        frac = perp_px / (w / 2.0)
        offset_m = height * frac * math.tan(self.HFOV / 2.0)
        if self.FLIP_LR:
            offset_m = -offset_m
        cross_track = -offset_m
        return heading_error, cross_track

    # ------------------------------------------------------------- the output

    def _publish(self, frame, fit, header):
        if fit is None:
            self.hits = 0
            self.misses += 1
            if self.detected and self.misses >= self.LOST_FRAMES:
                self.detected = False
                self.heading_error = None
                self.cross_track = None
                self.quality = 0.0
                self.get_logger().warning(
                    f"LINE LOST after {self.LOST_FRAMES} frames without it. "
                    "The flight node falls back to the latched bearing.")
            self.detected_pub.publish(Bool(data=self.detected))
            self._say("no line in frame.")
            self._draw(frame, None, header)
            return

        angle_img, perp_px, quality, w, h = fit
        heading_error, cross_track = self._to_body(angle_img, perp_px, w, h)

        self.misses = 0
        self.hits += 1
        a = self.SMOOTHING
        self.heading_error = (heading_error if self.heading_error is None
                              else a * heading_error + (1 - a) * self.heading_error)
        if cross_track is None:
            self.cross_track = None
        else:
            self.cross_track = (cross_track if self.cross_track is None
                                else a * cross_track + (1 - a) * self.cross_track)
        self.quality = quality

        if not self.detected and self.hits >= self.DETECT_FRAMES:
            self.detected = True
            self.get_logger().warning(
                f"LINE FOUND: {math.degrees(self.heading_error):+.1f} deg off the "
                f"nose, quality {quality:.2f}.")

        self.detected_pub.publish(Bool(data=self.detected))
        if self.detected:
            msg = Vector3Stamped()
            msg.header = header
            msg.vector.x = float(self.heading_error)
            # NaN, not 0.0: "I cannot measure this" and "you are exactly on the
            # line" are opposite instructions, and a zero would be obeyed.
            msg.vector.y = (float('nan') if self.cross_track is None
                            else float(self.cross_track))
            msg.vector.z = float(quality)
            self.track_pub.publish(msg)

        height, live = self._agl()
        self._say(
            f"line {math.degrees(self.heading_error):+.1f} deg, "
            + ("cross-track n/a (no height)" if self.cross_track is None
               else f"aircraft {self.cross_track:+.2f} m "
                    f"{'right' if self.cross_track >= 0 else 'left'} of it")
            + f", quality {quality:.2f}, "
            + (f"agl {height:.2f} m" + ("" if live else " (FALLBACK, not measured)")
               if height is not None
               else px4_height.why_no_height(self.estimator_flags,
                                             self.local_position))
            + f", {'DETECTED' if self.detected else 'settling'}")
        self._draw(frame, fit, header)

    def _say(self, text):
        self.info_pub.publish(String(data=text))
        self.get_logger().info(text, throttle_duration_sec=1.0)

    def _draw(self, frame, fit, header):
        if self.preview_pub is None and self.stream is None:
            return
        img = frame.copy()
        h, w = img.shape[:2]
        # The crosshair is straight down: put the line on it and the aircraft
        # is on the corridor.
        cv2.drawMarker(img, (w // 2, h // 2), (255, 255, 255),
                       cv2.MARKER_CROSS, 24, 1)
        if self._mask is not None and self._mask.shape[:2] == img.shape[:2]:
            # Tint what was segmented as the strip. On the bench this is the
            # thing to look at: if the tint is the granite, or half the pad,
            # strip_is_bright or the morphology kernels are wrong.
            tint = np.zeros_like(img)
            tint[:, :, 1] = self._mask
            img = cv2.addWeighted(img, 1.0, tint, 0.25, 0.0)
        if fit is not None:
            angle_img, perp_px, quality, _, _ = fit
            d = np.array([math.cos(angle_img), math.sin(angle_img)])
            n = np.array([-d[1], d[0]])
            centre = np.array([w / 2.0, h / 2.0])
            foot = centre - n * perp_px
            a = (foot - d * w).astype(int)
            b = (foot + d * w).astype(int)
            cv2.line(img, tuple(a), tuple(b), (0, 255, 0), 2)
            cv2.line(img, (w // 2, h // 2), tuple(foot.astype(int)),
                     (0, 200, 255), 2)
            txt = f"{math.degrees(self.heading_error):+.1f}deg"
            if self.cross_track is not None:
                txt += f" {self.cross_track:+.2f}m"
            txt += f" q{quality:.2f}"
            cv2.putText(img, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0) if self.detected else (0, 200, 255), 2)
        else:
            cv2.putText(img, "NO LINE", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), 2)
        if self.preview_pub is not None:
            self.preview_pub.publish(array_to_imgmsg(img, 'bgr8', header))
        if self.stream is not None:
            ok, buf = cv2.imencode('.jpg', img,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY])
            if ok:
                self._jpeg = buf.tobytes()


def main(args=None):
    rclpy.init(args=args)
    node = LineDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stream is not None:
            node.stream.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
