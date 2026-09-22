#!/usr/bin/env python3
"""HOT BOX -> LED -> SERVO, on the bench. No flying, no offboard, no arming.

This is the drop half of the mission with the flying cut out of it. You hold
the aircraft in your hands (or on a stick, or a rope) and lower it towards the
hot box yourself; this node watches the thermal camera, drives the LEDs the
same way the mission does, and fires the servo at the drop height. It never
publishes a setpoint, never asks for offboard mode and never arms anything, so
it is safe to run with the props off and the aircraft on the table.

WHAT IT DOES, IN ORDER

    nothing hot in frame ................. LED off,        "searching"
    hot blob found, off to one side ...... LED blink_blue,  "centre me"
    hot blob under the camera ............ LED blink_red,   "descending"
        (the same blink_red the real DESCEND stage uses)
    under it AND at drop_altitude ........ LED solid_green, servo OPEN
    servo_hold_seconds later ............. LED off,         servo neutral

It does not move the servo itself.  It publishes True on drop_trigger_topic
and servo_controller.py opens the servo, exactly as in flight -- so what you
are bench-testing is the real release path, not a stand-in for it.

THE DETECTION IS THE REAL DETECTION.  find_blobs() here is the same function
thermal_sensor draws its boxes with and thermal_drop flies to, with the same
min_contrast and min_blob_pixels.  If this node picks the wrong box, so would
the mission.

WATCHING IT

    http://<jetson>:8082/     thermal_sensor's MJPEG stream: the frame in
                              false colour, a THICK GREEN BOX on the blob
                              this node is acting on, a yellow cross on the
                              centroid it measures the offset from, and a
                              white crosshair at frame centre = straight
                              down.  Centring the green box on the crosshair
                              is the whole job.
    ros2 topic echo /thermal/bench     this node's running commentary.

HEIGHT

    Taken from VehicleLocalPosition.dist_bottom -- the downward rangefinder,
    which over a box reads height above the BOX TOP, which is the number that
    matters for a drop.  PX4's EKF runs while disarmed, so this works with the
    aircraft in your hands.  No rangefinder, or you just want to test the
    centring and the servo: require_altitude:=false and it fires on centring
    alone.

THE ONE PX4 GOTCHA

    PX4 IGNORES MAV_CMD_DO_SET_ACTUATOR WHILE DISARMED.  On the bench, run
    servo_controller with servo_command:=actuator_test and servo_function set
    to the output's FUNCTION number from QGC's Actuators tab -- that is the
    command QGC itself uses and it does work disarmed.  The bench launch file
    does this for you.

Run:
    ros2 launch drone_testing thermal_bench.launch.py servo_function:=<N>
"""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from drone_testing.thermal_common import H, W, find_blobs


class ThermalBench(Node):

    SEARCH, CENTRE, DESCEND, DROPPED, DONE = range(5)
    NAMES = ('SEARCH', 'CENTRE', 'DESCEND', 'DROPPED', 'DONE')

    def __init__(self):
        super().__init__('thermal_bench')

        def p(name, default):
            return self.declare_parameter(name, default).value

        # --- the same numbers the flight node uses, so this tests those ---
        self.MIN_CONTRAST = float(p('min_contrast', 3.0))
        self.MIN_BLOB_PIXELS = int(p('min_blob_pixels', 2))
        self.HFOV = math.radians(float(p('hfov_deg', 110.0)))
        self.VFOV = math.radians(float(p('vfov_deg', 75.0)))
        self.CAM_YAW = math.radians(float(p('cam_yaw_deg', 0.0)))
        self.FLIP_LR = bool(p('flip_lr', False))
        self.FLIP_UD = bool(p('flip_ud', False))
        self.DROP_ALTITUDE = float(p('drop_altitude', 0.50))
        self.ALTITUDE_TOLERANCE = float(p('altitude_tolerance', 0.15))
        self.CENTRE_TOLERANCE = float(p('centre_tolerance', 0.20))

        # --- bench-only behaviour ---
        self.REQUIRE_ALTITUDE = bool(p('require_altitude', True))
        self.HOLD_SECONDS = float(p('settle_seconds', 1.0))
        self.SERVO_HOLD_SECONDS = float(p('servo_hold_seconds', 2.0))
        self.REARM_SECONDS = float(p('rearm_seconds', 0.0))
        self.TOPIC = str(p('drop_trigger_topic', '/servo/drop'))
        self.LOST_SECONDS = float(p('lost_seconds', 1.0))

        self.drop_pub = self.create_publisher(Bool, self.TOPIC, 10)
        self.led_pub = self.create_publisher(String, 'led/command', 10)
        self.status_pub = self.create_publisher(String, 'thermal/bench', 10)

        # PX4's /fmu/out/... topics are BEST_EFFORT. A RELIABLE subscriber is
        # silently delivered nothing at all, and the height would read "no
        # rangefinder" for ever.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position',
                                 self.position_callback, px4_qos)
        self.create_subscription(Image, 'thermal/image', self.image_callback, 10)

        self.local_position = None
        self.stage = self.SEARCH
        self.stage_since = time.monotonic()
        self.in_band_since = None
        self.last_seen = 0.0
        self.dropped_at = None
        self.drops = 0
        self.led_mode = None
        self.last_log = 0.0
        self.best = None        # (peak_C, err_m or None, agl or None)

        self.create_timer(0.25, self.tick)
        self._set_led('off')

        self.get_logger().warning(
            "BENCH: no flying, no offboard, no arming. Lower the aircraft over "
            f"the hot box by hand. Drop at {self.DROP_ALTITUDE:.2f} m "
            f"(+/-{self.ALTITUDE_TOLERANCE:.2f}) with the box within "
            f"{self.CENTRE_TOLERANCE * 100:.0f} cm of straight down"
            + ("" if self.REQUIRE_ALTITUDE else ", HEIGHT CHECK OFF")
            + f". Watch it at http://<this host>:8082/ . Trigger on {self.TOPIC}.")

    # ------------------------------------------------------------- the inputs

    def position_callback(self, msg):
        self.local_position = msg

    def agl(self):
        lp = self.local_position
        if lp is None or not lp.dist_bottom_valid:
            return None
        return float(lp.dist_bottom)

    def pixel_ray_body(self, row, col):
        """Unit-less FRD ray through a pixel centre. Same as thermal_drop."""
        u = (col + 0.5 - W / 2.0) / (W / 2.0) * math.tan(self.HFOV / 2.0)
        v = (row + 0.5 - H / 2.0) / (H / 2.0) * math.tan(self.VFOV / 2.0)
        if self.FLIP_LR:
            u = -u
        if self.FLIP_UD:
            v = -v
        fwd, right = -v, u
        c, s = math.cos(self.CAM_YAW), math.sin(self.CAM_YAW)
        return np.array([fwd * c - right * s, fwd * s + right * c, 1.0])

    def image_callback(self, msg):
        if msg.encoding != '32FC1' or msg.width != W or msg.height != H:
            self.get_logger().error(
                f"thermal/image is {msg.width}x{msg.height} {msg.encoding}, "
                f"expected {W}x{H} 32FC1.", throttle_duration_sec=5.0)
            return
        grid = np.frombuffer(msg.data, dtype=np.float32).reshape(H, W)
        # find_blobs returns (blobs, ambient) -- the ambient is the median of
        # the frame, which is what every contrast threshold is measured from.
        blobs, ambient = find_blobs(grid, self.MIN_CONTRAST,
                                    min_pixels=self.MIN_BLOB_PIXELS)
        if not blobs:
            self.best = None
            return

        hot = blobs[0]          # find_blobs returns them hottest first
        # Offset of the blob from straight down, in metres on the ground.
        # Flat-and-level is the bench assumption: you are holding it level,
        # and there is no attitude to compensate for because nothing is
        # flying. In the air thermal_drop rotates this ray by the attitude
        # the frame was taken at -- that part is not what this tests.
        ray = self.pixel_ray_body(hot['row'], hot['col'])
        alt = self.agl()
        h = alt if alt is not None else 1.0
        err = h * float(np.hypot(ray[0], ray[1]))
        self.best = (float(hot['peak']), err, alt, len(blobs), ambient)
        self.last_seen = time.monotonic()

    # -------------------------------------------------------------- the logic

    def tick(self):
        now = time.monotonic()

        if self.stage == self.DONE:
            return

        if self.stage == self.DROPPED:
            if now - self.dropped_at >= self.SERVO_HOLD_SECONDS:
                self.drop_pub.publish(Bool(data=False))
                self._set_led('off')
                self.get_logger().warning(
                    "Servo back to neutral, LED off. Reload the cone.")
                if self.REARM_SECONDS > 0.0:
                    self._enter(self.SEARCH)
                    self.get_logger().info(
                        f"Re-arming in {self.REARM_SECONDS:.1f} s for another go.")
                    self.dropped_at = now       # reused as the cooldown mark
                    self.stage_since = now + self.REARM_SECONDS
                else:
                    self._enter(self.DONE)
                    self.get_logger().warning(
                        f"Bench run complete after {self.drops} drop(s). "
                        "rearm_seconds:=5.0 to loop instead of stopping.")
            return

        if self.stage_since > now:
            return              # cooldown between bench cycles

        stale = now - self.last_seen > self.LOST_SECONDS
        if self.best is None or stale:
            if self.stage != self.SEARCH:
                self.get_logger().info("Lost the hot box. Searching again.")
            self.in_band_since = None
            self._enter(self.SEARCH)
            self._set_led('off')
            self._say("SEARCH: nothing hot in frame.")
            return

        peak, err, alt, count, ambient = self.best
        centred = err <= self.CENTRE_TOLERANCE
        at_height = (alt is not None
                     and abs(alt - self.DROP_ALTITUDE) <= self.ALTITUDE_TOLERANCE)
        if not self.REQUIRE_ALTITUDE:
            at_height = True

        height_txt = (f"{alt:.2f} m" if alt is not None else
                      "no rangefinder" if self.REQUIRE_ALTITUDE else "height ignored")

        if not centred:
            self.in_band_since = None
            self._enter(self.CENTRE)
            self._set_led('blink_blue')
            self._say(f"CENTRE: hot box {peak:.1f} C, {err * 100:.0f} cm to one "
                      f"side (need {self.CENTRE_TOLERANCE * 100:.0f}), at "
                      f"{height_txt}, {count} blob(s), ambient "
                      f"{ambient:.1f} C. LED blue.")
            return

        # Centred. This is the point the mission calls DESCEND.
        if self.stage != self.DESCEND:
            self._enter(self.DESCEND)
            self._set_led('blink_red')
            self.get_logger().warning(
                f"DESCEND: hot box {peak:.1f} C is under the camera "
                f"({err * 100:.0f} cm off), at {height_txt}. LED blinking red. "
                f"Lower it to {self.DROP_ALTITUDE:.2f} m.")

        if not at_height:
            self.in_band_since = None
            self._say(f"DESCEND: over the box, {err * 100:.0f} cm off, at "
                      f"{height_txt}, want {self.DROP_ALTITUDE:.2f} m.")
            return

        if self.in_band_since is None:
            self.in_band_since = now
            self._say("DESCEND: in the drop band. Hold it there.")
            return
        if now - self.in_band_since < self.HOLD_SECONDS:
            return

        self.drops += 1
        self.dropped_at = now
        self.in_band_since = None
        self._enter(self.DROPPED)
        self._set_led('solid_green')
        self.drop_pub.publish(Bool(data=True))
        self.get_logger().warning(
            f"DROP #{self.drops}: {peak:.1f} C box, {err * 100:.0f} cm off "
            f"centre at {height_txt}. True published on {self.TOPIC}, LED "
            f"solid green, servo open for {self.SERVO_HOLD_SECONDS:.1f} s.")

    # --------------------------------------------------------------- plumbing

    def _enter(self, stage):
        if stage != self.stage:
            self.stage = stage
            self.stage_since = time.monotonic()

    def _set_led(self, mode):
        if mode == self.led_mode:
            return
        self.led_mode = mode
        self.led_pub.publish(String(data=mode))

    def _say(self, text):
        self.status_pub.publish(String(data=f"{self.NAMES[self.stage]}|{text}"))
        now = time.monotonic()
        if now - self.last_log >= 1.0:
            self.last_log = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = ThermalBench()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Leave nothing open and nothing glowing.
        node.drop_pub.publish(Bool(data=False))
        node.led_pub.publish(String(data='off'))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
