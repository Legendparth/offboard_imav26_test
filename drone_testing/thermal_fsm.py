#!/usr/bin/env python3
"""
The WHOLE thermal mission, as one state machine: pad -> marker -> boxes ->
drop -> marker -> land.

    ARM -> ground wait -> TAKEOFF to survey_altitude -> HOLD
      -> MARK_SEARCH   forward, on the arming heading, until the DOWNWARD
                       camera puts an ArUco marker under the aircraft
      -> MARK_HOVER    centre on that marker and hold mark_hover_seconds
      -> BOX_OFFSET    box_offset_right metres RIGHT, which is the step that
                       puts the boxes under the thermal camera and nothing
                       else
      -> SURVEY        (inherited) map every warm blob in NED
      -> APPROACH      (inherited) fly the DROP POINT over the hottest one
                       and confirm it is still the hottest thing in frame
      -> DESCEND       (inherited) step down only while centred, LED BLINKING
                       RED the whole way
      -> HOVER         (inherited) confirm alignment at drop_altitude, fire
                       the SERVO, LED off
      -> RETREAT       (inherited) climb to retreat_altitude, then step
                       retreat_right metres RIGHT
      -> LAND_SEARCH   creep forward until the downward camera finds the
                       LANDING marker
      -> LAND_CENTRE   hold over it until the estimate settles
      -> LAND_DESCEND  walk down, correcting off the marker, and hand the
                       last pad_handoff_height metres to PX4's land

    q -> abort into a controlled descent.   k -> force-disarm.

WHAT IS NEW HERE AND WHAT IS NOT
--------------------------------
Everything from SURVEY to RETREAT is thermal_drop.py, unchanged and not
reimplemented: the blob projection, the survey map, the hottest-box
verification, the descent gating, the servo, the LED, the failsafes. This
file is the two ENDS -- getting to the boxes, and landing on a marker
afterwards -- plus the plumbing that joins them.

The marker stages are course_fsm.py's pad stages in all but name. The same
nudge-a-fraction-of-the-error-at-a-time centring, the same "losing the
marker stops the descent rather than continuing blind", the same handoff
height. They are here rather than inherited because course_fsm is a
WindowTraverse and this is a ThermalDrop, and the two branches of the family
meet only at OffboardSequence.

WHY THERE IS A MARKER IN THE MIDDLE OF A THERMAL MISSION
--------------------------------------------------------
The boxes are the only thing this mission cares about, and the aircraft
cannot see them until it is nearly over them: an MLX90640 at survey altitude
covers a few metres, and dead reckoning from the takeoff pad over twenty is
not good enough to guarantee they are in that few. The marker on the floor
is the fix: it is a POSITION FIX the aircraft can measure, it sits at a
known offset from the boxes, and once the aircraft is centred on it the
remaining leg is one short sideways step whose length is a course
measurement rather than an accumulated error.

So MARK_SEARCH is not "find a marker", it is "re-zero". If it finds
nothing, the mission has no idea where the boxes are and says so -- see
mark_required.

EVERY LENGTH IN THIS FILE IS A COURSE LENGTH
--------------------------------------------
...which means the simulated arena, being the real one scaled by 2.2, needs
all of them multiplied by 2.2. thermal_drop_sitl.launch.py does exactly
that and nothing in this file knows about it; the defaults below are the
REAL course. See that file's header.

RUN IT
    Simulation, the whole thing:
        ros2 launch drone_testing thermal_drop_sitl.launch.py

    Hardware, with the keyboard aborts (a launched node has no tty):
        ros2 launch drone_testing thermal_drop.launch.py \\
            fsm:=true agent_only:=true
        ros2 run drone_testing thermal_fsm --ros-args -p ...
"""

import collections
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from px4_msgs.msg import VehicleStatus

from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.thermal_drop import ThermalDrop


class ThermalFSM(ThermalDrop):

    # ---- the stages this file adds ----------------------------------------
    MARK_SEARCH = "MARK_SEARCH"     # forward until a marker is underneath
    MARK_HOVER = "MARK_HOVER"       # centre on it, hold, then step across
    BOX_OFFSET = "BOX_OFFSET"       # the step that puts the boxes in frame
    LAND_SEARCH = "LAND_SEARCH"     # after the retreat: find the landing pad
    LAND_CENTRE = "LAND_CENTRE"     # settle over it
    LAND_DESCEND = "LAND_DESCEND"   # down on the marker, then PX4 lands

    FSM_STAGES = (MARK_SEARCH, MARK_HOVER, BOX_OFFSET,
                  LAND_SEARCH, LAND_CENTRE, LAND_DESCEND)

    # The inherited timer_callback, flight clock and status line all key off
    # DROP_STAGES, so extending it here is what makes the new stages
    # first-class: they get the setpoint stream, the failsafe checks, the
    # flight clock and the LCD line without any of that being repeated.
    DROP_STAGES = ThermalDrop.DROP_STAGES + FSM_STAGES

    # ---- the legs, in REAL course metres ----------------------------------
    MARK_SEARCH_DISTANCE = 11.0     # m of forward creep before giving up
    MARK_SEARCH_SPEED = 0.30        # m/s. Slow: a marker that goes through
                                    # the frame between two detector ticks
                                    # is a marker that was never there.
    MARK_MIN_TRAVEL = 1.00          # m that must be flown before a marker is
                                    # allowed to count. The aircraft arms ON
                                    # a marked pad, and without this the
                                    # mission would "find" the pad it is
                                    # standing on the instant it lifts off.
                                    # marker_ids on the detector is the other
                                    # half of that guard; this one works even
                                    # if every pad shares an id.
    MARK_HOVER_SECONDS = 1.0        # s stationary over the marker. Not a
                                    # length: NOT scaled.
    BOX_OFFSET_RIGHT = 1.50         # m RIGHT of the marker, onto the boxes.
    LAND_SEARCH_DISTANCE = 2.50     # m of forward creep looking for the pad
    LAND_SEARCH_SPEED = 0.30

    # ---- flying a straight line, and why it needs saying ------------------
    #
    #   A leg used to be flown by pointing the inherited carrot at a single
    #   point at the FAR END of it. That tracks a line badly, and the reason
    #   is pure geometry: the commanded direction is (target - here), so a
    #   cross-track error e with d metres still to run bends the command by
    #   only atan(e/d). At the start of the 21.5 m marker run, a whole metre
    #   off the line buys a 2.7 deg correction -- which loses to any
    #   disturbance, so the error simply persists until the last few metres,
    #   by which point the aircraft has already flown past the marker,
    #   offset sideways.
    #
    #   Measured: 1.09 m right over 19.7 m, and the marker only stayed in
    #   frame because the camera's footprint is 6.6 m wide.
    #
    #   So the carrot is now put ON the line, LEG_LOOKAHEAD ahead of where
    #   the aircraft projects onto it, and recomputed every tick. The same
    #   metre of cross-track error now bends the command by atan(e/1.54) =
    #   33 deg. The error is driven out in metres instead of tens of metres,
    #   and the leg is a LINE the aircraft is held on rather than a
    #   destination it eventually arrives near.
    LEG_LOOKAHEAD = 0.70        # m ahead along the line. Must stay LARGER
                                # than move_leash, or the carrot lands inside
                                # the leash radius, the leash clamps it, and
                                # the commanded direction stops meaning
                                # anything. It is a world distance, so it
                                # scales with the arena.
    LEG_TRIM_DEG = 0.0          # deg added to every leg heading, +ve to the
                                # RIGHT. The dial for a KNOWN, repeatable
                                # aim bias -- a mag offset on the real
                                # aircraft, say. Leave at 0 until a straight
                                # leg has been measured and found bent.
    LEG_SETTLE_SAMPLES = 20     # heading samples averaged at the end of the
                                # hold to fix the corridor direction. See
                                # _latch_leg_yaw().
    # An ABSOLUTE corridor bearing in the estimator's NED frame, in degrees,
    # or NaN to measure it from the settled heading instead.
    #
    #   The heading estimate and the POSITION frame do not share a north.
    #   EKF2 datums yaw off the magnetometer and its declination; x/y come
    #   from flow and the IMU. Measured in the scaled arena, with the
    #   airframe spawned exactly along the corridor and flying a line the
    #   tracker held to 3 cm: the estimator reported a heading of +5.95 deg,
    #   and the aircraft physically flew +5.93 deg off the corridor. The
    #   heading was steady to 0.03 deg. It was not noise -- it was a fixed
    #   5.95 deg offset between "where the nose is" and "what the heading
    #   says", and flying the leg on the heading inherits all of it: 2.40 m
    #   right over 23 m.
    #
    #   So where the corridor's bearing IS KNOWN in the estimator's frame,
    #   say it, and do not infer it. In the simulator it is known exactly:
    #   the arena's +y is the estimator's north and the aircraft is spawned
    #   along it, so the corridor is 0.0 deg. On the aircraft it is not
    #   known -- the operator aims the nose, there is no survey of the hall
    #   -- so it is NaN there and the settled heading is used, with
    #   LEG_TRIM to take out a declination once a leg has been flown and
    #   measured.
    LEG_BEARING_DEG = float('nan')

    # ---- the marker, and flying on it -------------------------------------
    PAD_CENTRE_TOLERANCE = 0.10     # m off the marker that counts as centred
    PAD_CENTRE_SECONDS = 1.0        # s it must stay there before descending
    PAD_DESCENT_RATE = 0.20         # m/s the setpoint walks down
    PAD_HANDOFF_HEIGHT = 0.45       # m at which PX4's land takes over. Below
                                    # this the marker no longer fits in the
                                    # frame, so there is nothing left to
                                    # correct on.
    PAD_GAIN = 0.8                  # fraction of the measured error moved per
                                    # tick. 1.0 chases noise; this converges
                                    # in a few ticks without ringing.
    PAD_MAX_NUDGE = 0.30            # m, the largest single step onto the
                                    # marker. A bad pose cannot throw the
                                    # aircraft further than this.
    PAD_LOST_SECONDS = 2.0          # s a pose stays usable after it arrives
    MARK_STAGE_TIMEOUT = 60.0
    PAD_STAGE_TIMEOUT = 45.0

    TIMER_PERIOD = 0.05             # s, OffboardSequence's loop period. The
                                    # marker descent walks commanded_altitude
                                    # down by rate * this every tick, so it
                                    # has to match the create_timer() in
                                    # offboard_sequence.py. A copy, because
                                    # the base class does not expose it;
                                    # course_fsm.py keeps the same copy.

    ARUCO_DETECT_TOPIC = '/aruco/detected'
    ARUCO_POINT_TOPIC = '/aruco/point'

    def __init__(self):
        super().__init__()
        n = self._declare_number

        self.MARK_SEARCH_DISTANCE = float(n('mark_search_distance',
                                            self.MARK_SEARCH_DISTANCE))
        self.MARK_SEARCH_SPEED = float(n('mark_search_speed',
                                         self.MARK_SEARCH_SPEED))
        self.MARK_MIN_TRAVEL = float(n('mark_min_travel', self.MARK_MIN_TRAVEL))
        self.MARK_HOVER_SECONDS = float(n('mark_hover_seconds',
                                          self.MARK_HOVER_SECONDS))
        self.BOX_OFFSET_RIGHT = float(n('box_offset_right', self.BOX_OFFSET_RIGHT))
        self.LAND_SEARCH_DISTANCE = float(n('land_search_distance',
                                            self.LAND_SEARCH_DISTANCE))
        self.LAND_SEARCH_SPEED = float(n('land_search_speed',
                                         self.LAND_SEARCH_SPEED))
        self.LEG_LOOKAHEAD = float(n('leg_lookahead', self.LEG_LOOKAHEAD))
        self.LEG_TRIM = math.radians(float(n('leg_trim_deg', self.LEG_TRIM_DEG)))
        self.LEG_BEARING = math.radians(float(n('leg_bearing_deg',
                                                self.LEG_BEARING_DEG)))
        if self.LEG_LOOKAHEAD <= self.MOVE_LEASH:
            self.get_logger().error(
                f"leg_lookahead {self.LEG_LOOKAHEAD:.2f} m is not bigger than "
                f"move_leash {self.MOVE_LEASH:.2f} m. The carrot would sit "
                "inside the leash radius, the leash would clamp it, and the "
                "commanded direction would stop tracking the line. Raising "
                f"leg_lookahead to {self.MOVE_LEASH * 1.5:.2f} m.")
            self.LEG_LOOKAHEAD = self.MOVE_LEASH * 1.5
        self.MARK_STAGE_TIMEOUT = float(n('mark_stage_timeout',
                                          self.MARK_STAGE_TIMEOUT))
        self.PAD_STAGE_TIMEOUT = float(n('pad_stage_timeout',
                                         self.PAD_STAGE_TIMEOUT))
        self.PAD_CENTRE_TOLERANCE = float(n('pad_centre_tolerance',
                                            self.PAD_CENTRE_TOLERANCE))
        self.PAD_CENTRE_SECONDS = float(n('pad_centre_seconds',
                                          self.PAD_CENTRE_SECONDS))
        self.PAD_DESCENT_RATE = float(n('pad_descent_rate', self.PAD_DESCENT_RATE))
        self.PAD_HANDOFF_HEIGHT = float(n('pad_handoff_height',
                                          self.PAD_HANDOFF_HEIGHT))
        self.PAD_GAIN = float(n('pad_gain', self.PAD_GAIN))
        self.PAD_MAX_NUDGE = float(n('pad_max_nudge', self.PAD_MAX_NUDGE))
        self.PAD_LOST_SECONDS = float(n('pad_lost_seconds', self.PAD_LOST_SECONDS))

        # false = no marker hunt at all: hold, then survey from the takeoff
        # point, which is thermal_drop.py's own mission. It is how you fly
        # this node over a box that is already underneath you.
        self.MARK_SEARCH_ON = bool(self.declare_parameter('mark_search', True).value)
        # true = a marker that is never found ENDS the mission (land where we
        # are) instead of surveying from wherever the creep gave up. Default
        # true because a survey from an unknown place is a survey of the
        # floor, and finding "no warm box" there teaches nothing.
        self.MARK_REQUIRED = bool(self.declare_parameter('mark_required', True).value)
        # false = plain PX4 land after the retreat, exactly as thermal_drop
        # has always done. true = the LAND_* stages below.
        self.PRECISION_LAND = bool(self.declare_parameter('precision_land', True).value)

        self.create_subscription(
            Bool, str(self.declare_parameter(
                'aruco_detect_topic', self.ARUCO_DETECT_TOPIC).value),
            self.aruco_detected_callback, 10, callback_group=self.sensor_cbg)
        self.create_subscription(
            PointStamped, str(self.declare_parameter(
                'aruco_point_topic', self.ARUCO_POINT_TOPIC).value),
            self.aruco_point_callback, 10, callback_group=self.sensor_cbg)

        self.aruco_flag = False
        self.aruco_point = None
        self.aruco_point_time = None
        self.aruco_id = None
        self.aruco_seen = 0

        self.mark_hover_since = None
        self.mark_xy = None             # the marker the boxes are measured off
        self.pad_settle_since = None
        self.pad_lost_since = None

        # The corridor direction, latched once at the end of the hold. None
        # until then; see _latch_leg_yaw() for why it is not home_yaw.
        self.leg_yaw = None
        self.leg_samples = collections.deque(maxlen=self.LEG_SETTLE_SAMPLES)
        # The leg currently being flown: start point, unit vector, length.
        self.leg_start = None
        self.leg_unit = None
        self.leg_heading = None
        self.leg_length = 0.0
        self.leg_cross = 0.0            # signed, +ve = RIGHT of the line
        self.leg_worst_cross = 0.0

        if self.MODE == 'fly':
            self.get_logger().warning(
                "THERMAL FSM: "
                + (f"forward up to {self.MARK_SEARCH_DISTANCE:.2f} m at "
                   f"{self.MARK_SEARCH_SPEED:.2f} m/s to the marker, hover "
                   f"{self.MARK_HOVER_SECONDS:.1f} s, "
                   f"{self.BOX_OFFSET_RIGHT:.2f} m RIGHT onto the boxes, then "
                   if self.MARK_SEARCH_ON else "no marker hunt; ")
                + "the thermal survey, the drop, the retreat, and "
                + (f"a precision landing on the next marker (creep up to "
                   f"{self.LAND_SEARCH_DISTANCE:.2f} m, centre to "
                   f"{self.PAD_CENTRE_TOLERANCE * 100:.0f} cm, down at "
                   f"{self.PAD_DESCENT_RATE:.2f} m/s, PX4 lands the last "
                   f"{self.PAD_HANDOFF_HEIGHT:.2f} m)."
                   if self.PRECISION_LAND else "a plain landing."))

    # ------------------------------------------------------------- the marker

    def aruco_detected_callback(self, msg):
        self.aruco_flag = bool(msg.data)

    def aruco_point_callback(self, msg):
        """The marker in the CAMERA body frame -> (forward, right) in metres.

        aruco_pose publishes +x right in the image, +y up in the image and +z
        opposite to where the camera looks. With the camera pointing down and
        its image-up towards the nose, that is forward = y, right = x, and
        height = -z. Only forward and right are taken: the height from a
        marker is scaled by whatever error the quoted field of view carries,
        and the rangefinder knows better. (down_cam.xacro says the same thing
        from the other end.)
        """
        self.aruco_seen += 1
        self.aruco_point = (float(msg.point.y), float(msg.point.x))
        self.aruco_point_time = time.monotonic()
        # aruco_pose puts the marker id after the frame name; a mission that
        # visits two pads wants to know which one it is looking at.
        fid = str(msg.header.frame_id)
        self.aruco_id = fid.split('/')[-1] if '/' in fid else None

    def _aruco_fresh(self):
        return (self.aruco_point is not None
                and self.aruco_point_time is not None
                and time.monotonic() - self.aruco_point_time <= self.PAD_LOST_SECONDS)

    def _aruco_ned(self):
        """The marker as an NED offset from the aircraft, or None."""
        if not self._aruco_fresh():
            return None
        lp = self.local_position
        if lp is None:
            return None
        forward, right = self.aruco_point
        h = lp.heading
        # forward along the heading, right 90 degrees clockwise from it
        return np.array([forward * math.cos(h) + right * math.sin(h),
                         forward * math.sin(h) - right * math.cos(h)])

    def _nudge_onto_marker(self):
        """Walk the hold point onto the marker, a fraction of the error a tick.

        Returns the size of the error it just measured, or None when there is
        no fresh pose -- which the callers treat as "stop, do not guess".
        """
        offset = self._aruco_ned()
        if offset is None:
            return None
        lp = self.local_position
        step = offset * self.PAD_GAIN
        n = float(np.linalg.norm(step))
        if n > self.PAD_MAX_NUDGE:
            step = step / n * self.PAD_MAX_NUDGE
        self._move_to(np.array([lp.x, lp.y]) + step)
        return float(np.linalg.norm(offset))

    def _marker_is_under_us(self):
        return self.aruco_flag and self._aruco_fresh()

    # ------------------------------------------------------------- THE LEGS

    def _latch_leg_yaw(self):
        """Fix the corridor direction, ONCE, from a settled heading.

        NOT home_yaw. home_yaw is a single sample of
        VehicleLocalPosition.heading taken at ARMING, and at arming EKF2's
        yaw is still converging: measured over a full run, with the airframe
        bolted to a spawn pose and not moving, it read +6.0 deg at arming and
        +3.2 deg ten seconds later, having wandered between +1.7 and +8.2.
        Aiming a 21.5 m leg with the arming sample put it 2.8 deg off the
        corridor before the aircraft had moved a centimetre.

        So the direction is taken at the END of the hold instead -- the
        aircraft is stationary, the estimator has had the whole climb and
        hold to settle -- and from the MEDIAN of the last LEG_SETTLE_SAMPLES
        readings rather than one of them, so a single outlier cannot set the
        course. The median is taken on the angle unwrapped about the newest
        sample, which is what keeps it honest across the +/-pi seam.

        LEG_TRIM is added on top: the dial for a bias that is known and
        repeatable rather than noisy.
        """
        lp = self.local_position
        samples = list(self.leg_samples) or ([lp.heading] if lp else [self.home_yaw])
        ref = samples[-1]
        unwrapped = [ref + wrap_pi(h - ref) for h in samples]
        median = float(np.median(unwrapped))
        spread = math.degrees(max(unwrapped) - min(unwrapped))
        measured = wrap_pi(median + self.LEG_TRIM)

        if not math.isnan(self.LEG_BEARING):
            self.leg_yaw = wrap_pi(self.LEG_BEARING)
            self.get_logger().warning(
                f"LEG HEADING: {math.degrees(self.leg_yaw):+.2f} deg, GIVEN as "
                "leg_bearing_deg, not measured. The settled heading reads "
                f"{math.degrees(measured):+.2f} deg (spread {spread:.2f} deg "
                f"over {len(samples)} samples), so the estimator's yaw datum "
                f"is {math.degrees(wrap_pi(measured - self.leg_yaw)):+.2f} deg "
                "off the corridor -- which is exactly the error that would "
                "have been flown.")
            return

        self.leg_yaw = measured
        self.get_logger().warning(
            f"LEG HEADING: {math.degrees(self.leg_yaw):+.2f} deg, the median "
            f"of {len(samples)} settled readings (spread {spread:.2f} deg"
            + (f", trim {math.degrees(self.LEG_TRIM):+.2f} deg" if self.LEG_TRIM else "")
            + f"). The arming heading was {math.degrees(self.home_yaw):+.2f} deg; "
            "every straight leg is flown off the number above, not that one. "
            "If the leg comes out bent, the log prints its cross-track error "
            "-- put the correction in leg_trim_deg.")

    def _begin_leg(self, heading, length, speed):
        """Start following a straight LINE from here, on an absolute heading."""
        lp = self.local_position
        self.leg_start = np.array([lp.x, lp.y])
        self.leg_heading = wrap_pi(heading)
        self.leg_unit = np.array([math.cos(self.leg_heading),
                                  math.sin(self.leg_heading)])
        self.leg_length = float(length)
        self.leg_cross = 0.0
        self.leg_worst_cross = 0.0
        self.MOVE_SPEED = speed

    def _follow_leg(self):
        """Put the carrot ON the line, a lookahead ahead of us, every tick.

        Returns (along, cross): how far down the line we have come, and how
        far off it we are -- signed, positive to the RIGHT of the direction
        of travel, which is the sign the logs and the status line use.

        The carrot is clamped to the end of the line, so the last lookahead
        metres close on the end point instead of overshooting it.
        """
        lp = self.local_position
        here = np.array([lp.x, lp.y])
        d = here - self.leg_start
        u = self.leg_unit
        along = float(d @ u)
        # In NED (x north, y east) a heading of h has unit (cos h, sin h),
        # and RIGHT of it is h + 90 deg, i.e. (-sin h, cos h) = (-u_y, u_x).
        # So this is positive when the aircraft is right of the line.
        right = np.array([-u[1], u[0]])
        cross = float(d @ right)
        self.leg_cross = cross
        if abs(cross) > abs(self.leg_worst_cross):
            self.leg_worst_cross = cross
        carrot = self.leg_start + u * min(along + self.LEG_LOOKAHEAD,
                                          self.leg_length)
        self._move_to(carrot)
        # Hold the nose on the CORRIDOR -- leg_yaw -- and NOT on this leg's
        # own direction. BOX_OFFSET runs 90 deg across the corridor, and
        # aiming the nose along it would spin the whole airframe through a
        # right angle for a 3.3 m sidestep: wasted time, and it turns the
        # thermal and down cameras away from the orientation the survey ring
        # and the marker conventions are written for.
        #
        # Re-aimed every tick rather than left alone, because yaw_setpoint
        # was latched from the ARMING heading and PX4 physically rotates the
        # airframe until the ESTIMATE matches it: true yaw in Gazebo walked
        # 2.4 deg right over one run doing exactly that.
        self._aim_yaw_at(self.leg_yaw)
        return along, cross

    def _aim_yaw_at(self, heading):
        """Walk the commanded yaw towards an absolute heading.

        Rewritten every tick rather than latched once, so the nose follows a
        heading that is still being refined instead of one measured once and
        then trusted for ever. yaw_remaining is what the inherited ramp
        consumes, and the ramp still limits the rate.
        """
        self.yaw_remaining = wrap_pi(heading - self.yaw_setpoint)

    def _leg_report(self, name):
        return (f"{name}: {self.leg_cross:+.2f} m "
                f"{'right' if self.leg_cross >= 0 else 'left'} of the line "
                f"(worst {self.leg_worst_cross:+.2f} m)")

    # ------------------------------------------------------ the state machine

    def _stage_handlers(self):
        handlers = super()._stage_handlers()
        handlers.update({
            self.MARK_SEARCH: self._handle_mark_search,
            self.MARK_HOVER: self._handle_mark_hover,
            self.BOX_OFFSET: self._handle_box_offset,
            self.LAND_SEARCH: self._handle_land_search,
            self.LAND_CENTRE: self._handle_land_centre,
            self.LAND_DESCEND: self._handle_land_descend,
        })
        return handlers

    def _handle_hold(self):
        """The post-takeoff hover, with the marker hunt in front of the survey.

        The inherited hold goes straight to SURVEY when it runs out, which is
        right for an aircraft that took off next to the boxes and wrong for
        one that took off twenty metres away from them.
        """
        if not self.MARK_SEARCH_ON:
            super()._handle_hold()
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        # Collect heading while STATIONARY and settled. This is the sample
        # set _latch_leg_yaw() takes the corridor direction from; the whole
        # point is that it is measured here and not at arming.
        lp = self.local_position
        if lp is not None:
            self.leg_samples.append(float(lp.heading))
        if self._in_stage_for() < self.HOLD_SECONDS or not self.hold_xy:
            self.get_logger().info(
                "Holding before the marker search"
                + ('' if self.hold_xy else ' (waiting for flow x/y latch)') + "...",
                throttle_duration_sec=1.0)
            return
        self._latch_leg_yaw()
        self._begin_mark_search()

    # ----------------------------------------------------------- MARK_SEARCH

    def _begin_mark_search(self):
        self._enter_stage(self.MARK_SEARCH)
        self._set_altitude(self.SURVEY_ALTITUDE)
        self._begin_leg(self.leg_yaw, self.MARK_SEARCH_DISTANCE,
                        self.MARK_SEARCH_SPEED)
        self.get_logger().warning(
            f"MARK_SEARCH: forward at {self.MARK_SEARCH_SPEED:.2f} m/s, up to "
            f"{self.MARK_SEARCH_DISTANCE:.2f} m, at "
            f"{self.SURVEY_ALTITUDE:.2f} m, on a LINE at "
            f"{math.degrees(self.leg_heading):+.2f} deg held to "
            f"{self.LEG_LOOKAHEAD:.2f} m lookahead, until the downward camera "
            f"puts a marker under us. The first {self.MARK_MIN_TRAVEL:.2f} m "
            "do not count -- that is the pad we armed on.")

    def _handle_mark_search(self):
        along, cross = self._follow_leg()
        if self._marker_is_under_us() and along >= self.MARK_MIN_TRAVEL:
            self._begin_mark_hover(along)
            return
        timed_out = self._in_stage_for() > self.MARK_STAGE_TIMEOUT
        if along >= self.MARK_SEARCH_DISTANCE - self.MOVE_TOLERANCE or timed_out:
            self._no_marker(along, timed_out)
            return
        self.get_logger().info(
            f"MARK_SEARCH: {along:.2f}/{self.MARK_SEARCH_DISTANCE:.2f} m, "
            f"{cross:+.2f} m off the line, "
            f"marker {'YES' if self.aruco_flag else 'no'} "
            f"({self.aruco_seen} poses seen).",
            throttle_duration_sec=1.0)

    def _no_marker(self, gone, timed_out):
        """The creep finished with nothing under the camera.

        Two honest endings, and the parameter picks which. Surveying from
        here is a survey of whatever happens to be below, which is normally
        bare floor; the default says so and lands instead of pretending the
        mission is still on its rails.
        """
        why = (f"no marker in {gone:.2f} m"
               + (" (timed out)" if timed_out else ""))
        if self.MARK_REQUIRED:
            self.outcome = f"ENDED: {why}; the boxes were never located."
            self.get_logger().error(
                f"MARK_SEARCH: {why}. The boxes are measured off that marker, "
                "so there is nowhere to survey from. Landing here. "
                "mark_required:=false surveys from here instead.")
            self._finish(why, land=True)
            return
        self.get_logger().error(
            f"MARK_SEARCH: {why}. mark_required is false, so surveying from "
            "here -- this is only useful if the boxes happen to be below.")
        self.moving = False
        self._begin_survey(np.array([self.local_position.x, self.local_position.y]))

    # ------------------------------------------------------------ MARK_HOVER

    def _begin_mark_hover(self, gone):
        self.MOVE_SPEED = self.MARK_SEARCH_SPEED
        self.mark_hover_since = None
        self._enter_stage(self.MARK_HOVER)
        self.get_logger().warning(
            f"MARK_HOVER: marker"
            + (f" (id {self.aruco_id})" if self.aruco_id else "")
            + f" found after {gone:.2f} m. "
            + self._leg_report("track") + ". Centring on it, then holding "
            f"{self.MARK_HOVER_SECONDS:.1f} s.")

    def _handle_mark_hover(self):
        error = self._nudge_onto_marker()
        now = time.monotonic()

        if error is None:
            # Lost mid-hover. Stop where we are rather than drift: the hold
            # point is already on the marker to within the last correction.
            self.moving = False
            if self._in_stage_for() > self.MARK_STAGE_TIMEOUT:
                self.get_logger().warning(
                    "MARK_HOVER: marker gone and not coming back. Stepping "
                    "across on the last fix anyway.")
                self._begin_box_offset()
            return

        # The clock starts when the aircraft is ON the marker, not when the
        # stage does. "Hover over the marker for a second" means a second of
        # being over it.
        if error > self.PAD_CENTRE_TOLERANCE:
            self.mark_hover_since = None
            if self._in_stage_for() > self.MARK_STAGE_TIMEOUT:
                self.get_logger().warning(
                    f"MARK_HOVER: still {error:.2f} m off after "
                    f"{self.MARK_STAGE_TIMEOUT:.0f} s. Stepping across anyway.")
                self._begin_box_offset()
                return
            self.get_logger().info(f"MARK_HOVER: {error:.2f} m off the marker.",
                                   throttle_duration_sec=1.0)
            return

        if self.mark_hover_since is None:
            self.mark_hover_since = now
            self.get_logger().warning(
                f"MARK_HOVER: centred to {error:.2f} m. Holding "
                f"{self.MARK_HOVER_SECONDS:.1f} s.")
        elif now - self.mark_hover_since >= self.MARK_HOVER_SECONDS:
            self._begin_box_offset()

    # ------------------------------------------------------------ BOX_OFFSET

    def _begin_box_offset(self):
        """One step RIGHT of the marker, which is where the boxes are.

        Off the ARMING heading, not the current one, and from the position the
        aircraft is holding NOW -- which MARK_HOVER has just put on the
        marker. That is the whole point of the hover: this leg starts from a
        measured place, so its end is a measured place too.
        """
        lp = self.local_position
        self.mark_xy = np.array([lp.x, lp.y])
        self._enter_stage(self.BOX_OFFSET)
        self._set_altitude(self.SURVEY_ALTITUDE)
        # 90 deg RIGHT of the corridor, and flown as a LINE like every other
        # leg, so this step lands square beside the marker instead of
        # somewhere on an arc through it. A negative offset just runs the
        # line backwards.
        across = wrap_pi(self.leg_yaw + math.copysign(
            math.pi / 2.0, self.BOX_OFFSET_RIGHT or 1.0))
        self._begin_leg(across, abs(self.BOX_OFFSET_RIGHT),
                        self.MARK_SEARCH_SPEED)
        end = self.leg_start + self.leg_unit * self.leg_length
        self.get_logger().warning(
            f"BOX_OFFSET: {abs(self.BOX_OFFSET_RIGHT):.2f} m "
            f"{'RIGHT' if self.BOX_OFFSET_RIGHT >= 0.0 else 'LEFT'} of the "
            f"marker, to ({end[0]:+.2f}, {end[1]:+.2f}) NED, on a line at "
            f"{math.degrees(self.leg_heading):+.2f} deg. The thermal survey "
            "starts there.")

    def _handle_box_offset(self):
        along, cross = self._follow_leg()
        left = self.leg_length - along
        timed_out = self._in_stage_for() > self.MARK_STAGE_TIMEOUT
        if left <= self.MOVE_TOLERANCE or timed_out:
            self.moving = False
            lp = self.local_position
            centre = np.array([lp.x, lp.y])
            self.get_logger().warning(
                f"BOX_OFFSET: over the boxes"
                f"{' (timed out)' if timed_out else ''}, {left:.2f} m short, "
                + self._leg_report("track")
                + f". {self.frames_seen} thermal frames so far.")
            self._begin_survey(centre)
            return
        self.get_logger().info(
            f"BOX_OFFSET: {left:.2f} m to go, {cross:+.2f} m off the line.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------- after the drop: LAND_*

    def _after_retreat(self):
        if not self.PRECISION_LAND:
            super()._after_retreat()
            return
        self._begin_land_search()

    def _begin_land_search(self):
        self._enter_stage(self.LAND_SEARCH)
        self._begin_leg(self.leg_yaw, self.LAND_SEARCH_DISTANCE,
                        self.LAND_SEARCH_SPEED)
        self.get_logger().warning(
            f"LAND_SEARCH: cone dropped and clear. Forward at "
            f"{self.LAND_SEARCH_SPEED:.2f} m/s, up to "
            f"{self.LAND_SEARCH_DISTANCE:.2f} m, on a line at "
            f"{math.degrees(self.leg_heading):+.2f} deg, until the landing "
            "marker is under us.")

    def _handle_land_search(self):
        gone, cross = self._follow_leg()
        if self._marker_is_under_us():
            self._begin_land_centre()
            return
        timed_out = self._in_stage_for() > self.PAD_STAGE_TIMEOUT
        if gone >= self.LAND_SEARCH_DISTANCE - self.MOVE_TOLERANCE or timed_out:
            # NOT a failure. The cone is in the box; there is simply no marker
            # here to land ON, and landing where we are is the right ending.
            self.outcome = ("MISSION COMPLETE: cone dropped on the hottest "
                            f"box. No landing marker in {gone:.2f} m of "
                            f"looking ({self.aruco_seen} poses seen).")
            self.get_logger().warning(
                f"LAND_SEARCH: no marker in {gone:.2f} m"
                f"{' (timed out)' if timed_out else ''}. The drop is done; "
                "landing here.")
            self._finish("drop complete, no marker to land on", land=True)
            return
        self.get_logger().info(
            f"LAND_SEARCH: {gone:.2f}/{self.LAND_SEARCH_DISTANCE:.2f} m, "
            f"{cross:+.2f} m off the line, "
            f"marker {'YES' if self.aruco_flag else 'no'}.",
            throttle_duration_sec=1.0)

    def _begin_land_centre(self):
        self.MOVE_SPEED = self.LAND_SEARCH_SPEED
        self.pad_settle_since = None
        self._enter_stage(self.LAND_CENTRE)
        self._nudge_onto_marker()
        self.get_logger().warning(
            "LAND_CENTRE: landing marker"
            + (f" (id {self.aruco_id})" if self.aruco_id else "")
            + " found. Centring over it.")

    def _handle_land_centre(self):
        error = self._nudge_onto_marker()
        now = time.monotonic()
        if error is None:
            self.pad_settle_since = None
            self.moving = False
            if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
                self.get_logger().error(
                    "LAND_CENTRE: lost the marker while centring. Landing here.")
                self._finish("lost the marker while centring", land=True)
            return
        if error <= self.PAD_CENTRE_TOLERANCE:
            if self.pad_settle_since is None:
                self.pad_settle_since = now
            elif now - self.pad_settle_since >= self.PAD_CENTRE_SECONDS:
                self._begin_land_descend()
            return
        self.pad_settle_since = None
        if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
            self.get_logger().warning(
                f"LAND_CENTRE: still {error:.2f} m off after "
                f"{self.PAD_STAGE_TIMEOUT:.0f} s. Going down anyway.")
            self._begin_land_descend()
            return
        self.get_logger().info(f"LAND_CENTRE: {error:.2f} m off the marker.",
                               throttle_duration_sec=1.0)

    def _begin_land_descend(self):
        self.pad_lost_since = None
        self._enter_stage(self.LAND_DESCEND)
        self.get_logger().warning(
            f"LAND_DESCEND: down at {self.PAD_DESCENT_RATE:.2f} m/s, "
            f"correcting off the marker, until {self.PAD_HANDOFF_HEIGHT:.2f} m.")

    def _handle_land_descend(self):
        """Walk the setpoint down while the marker keeps saying where it is.

        Losing it STOPS the descent and holds; it does not keep going blind.
        Regaining it resumes. Below the handoff height the marker no longer
        fits in the frame, so PX4's land takes the last part.
        """
        now = time.monotonic()
        alt = self.relative_altitude()
        error = self._nudge_onto_marker()

        if error is None:
            self.moving = False
            if self.pad_lost_since is None:
                self.pad_lost_since = now
                self.get_logger().error(
                    "LAND_DESCEND: marker lost. Holding altitude until it is back.")
            if now - self.pad_lost_since > self.PAD_STAGE_TIMEOUT:
                self.get_logger().error(
                    "LAND_DESCEND: the marker never came back. Landing from here.")
                self._finish("marker lost during the descent", land=True)
            return
        if self.pad_lost_since is not None:
            self.get_logger().warning("LAND_DESCEND: marker back; resuming.")
            self.pad_lost_since = None

        if alt is not None and alt <= self.PAD_HANDOFF_HEIGHT:
            self.outcome = ("MISSION COMPLETE: cone dropped on the hottest box, "
                            f"landed on the marker {error:.2f} m off centre")
            self._finish("over the marker, handing the last "
                         f"{self.PAD_HANDOFF_HEIGHT:.2f} m to PX4", land=True)
            return

        # DROP_ALTITUDE is the floor _set_altitude() clamps to, and it is
        # above the handoff height, so the descent is walked directly here.
        self.commanded_altitude = max(
            self.PAD_HANDOFF_HEIGHT,
            self.commanded_altitude - self.PAD_DESCENT_RATE * self.TIMER_PERIOD)
        self.target_z = self.home_z - self.commanded_altitude
        self.get_logger().info(
            f"LAND_DESCEND: alt {'n/a' if alt is None else f'{alt:.2f}'} -> "
            f"{self.commanded_altitude:.2f} m, {error:.2f} m off the marker.",
            throttle_duration_sec=1.0)

    # ---------------------------------------------------------------- status

    def publish_status(self):
        if self.current_stage not in self.FSM_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.current_stage in (self.MARK_SEARCH, self.LAND_SEARCH,
                                  self.BOX_OFFSET):
            # How far down the leg, and how far OFF it. The cross-track
            # number is the one worth a place on a five-field LCD line: it is
            # what says the aircraft is flying the corridor and not drifting
            # across it.
            along = 0.0
            lp = self.local_position
            if lp is not None and self.leg_start is not None:
                along = float((np.array([lp.x, lp.y]) - self.leg_start)
                              @ self.leg_unit)
            detail = f"go{along:.1f} x{self.leg_cross:+.2f}"
        else:
            err = self._aruco_ned()
            detail = ('mk?' if err is None
                      else f"mk{float(np.linalg.norm(err)):.2f}")
        self.status_pub.publish(String(data="|".join([
            self.current_stage, 'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail])))

    def _on_heading_reset(self, delta):
        """EKF2 has moved its idea of north; move the leg with it.

        The base class shifts home_yaw and yaw_setpoint, and ThermalDrop
        turns the thermal map. The LEG is this class's own piece of NED
        state and nothing else knows about it: leave it alone through a
        reset and the corridor direction silently becomes wrong by delta,
        and the aircraft flies the rest of a 21 m run at an angle for a
        reason that appears nowhere in the log.

        The line is rotated about the AIRCRAFT, not about its own start
        point, because the reset did not move the aircraft -- it re-labelled
        the frame around it. Rotating about the start would drag the line
        sideways as well as turning it.
        """
        super()._on_heading_reset(delta)
        if abs(delta) < 1e-9:
            return
        self.leg_samples = collections.deque(
            (wrap_pi(h + delta) for h in self.leg_samples),
            maxlen=self.LEG_SETTLE_SAMPLES)
        if self.leg_yaw is not None:
            self.leg_yaw = wrap_pi(self.leg_yaw + delta)
        if self.leg_heading is None or self.leg_start is None:
            return
        lp = self.local_position
        self.leg_heading = wrap_pi(self.leg_heading + delta)
        self.leg_unit = np.array([math.cos(self.leg_heading),
                                  math.sin(self.leg_heading)])
        if lp is not None:
            pivot = np.array([lp.x, lp.y])
            c, sn = math.cos(delta), math.sin(delta)
            rot = np.array([[c, -sn], [sn, c]])
            self.leg_start = pivot + rot @ (self.leg_start - pivot)
        if self.mark_xy is not None and lp is not None:
            pivot = np.array([lp.x, lp.y])
            c, sn = math.cos(delta), math.sin(delta)
            rot = np.array([[c, -sn], [sn, c]])
            self.mark_xy = pivot + rot @ (self.mark_xy - pivot)
        self.get_logger().warning(
            f"Leg turned with the frame: now "
            f"{math.degrees(self.leg_heading):+.2f} deg.")

    def destroy_node(self):
        self.get_logger().warning(
            f"Thermal FSM: {self.aruco_seen} marker poses seen."
            + (f" Worst cross-track on the last leg: "
               f"{self.leg_worst_cross:+.2f} m." if self.leg_start is not None
               else ""))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ThermalFSM()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
