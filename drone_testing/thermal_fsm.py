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
from geometry_msgs.msg import PointStamped, Vector3Stamped
from std_msgs.msg import Bool, Float32, String
from px4_msgs.msg import VehicleStatus

from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.thermal_drop import ThermalDrop, quat_rotate


class ThermalFSM(ThermalDrop):

    # ---- the stages this file adds ----------------------------------------
    MARK_SEARCH = "MARK_SEARCH"     # forward until a marker is underneath
    MARK_HOVER = "MARK_HOVER"       # centre on it, hold, then step across
    BOX_OFFSET = "BOX_OFFSET"       # the step that puts the boxes in frame
    LAND_SEARCH = "LAND_SEARCH"     # after the retreat: find the FIRST marker
    LAND_ALIGN = "LAND_ALIGN"       # square up on it -- it is a datum, not a pad
    LAND_RETURN = "LAND_RETURN"     # back down the corridor to the landing pad
    LAND_CENTRE = "LAND_CENTRE"     # settle over it
    LAND_DESCEND = "LAND_DESCEND"   # down on the marker, then PX4 lands

    FSM_STAGES = (MARK_SEARCH, MARK_HOVER, BOX_OFFSET, LAND_SEARCH,
                  LAND_ALIGN, LAND_RETURN, LAND_CENTRE, LAND_DESCEND)

    # The inherited timer_callback, flight clock and status line all key off
    # DROP_STAGES, so extending it here is what makes the new stages
    # first-class: they get the setpoint stream, the failsafe checks, the
    # flight clock and the LCD line without any of that being repeated.
    DROP_STAGES = ThermalDrop.DROP_STAGES + FSM_STAGES

    # ---- the legs, in REAL course metres ----------------------------------
    MARK_SEARCH_DISTANCE = 9.00     # m of forward creep before giving up.
                                    # MEASURED: the takeoff marker and the
                                    # marker in front of it are 8.7 m to
                                    # 8.8 m apart in the real arena, so 9 m
                                    # is that distance plus the slack for
                                    # where the aircraft actually left the
                                    # pad. It is a CAP, not a leg: a marker
                                    # seen at 8.2 m stops the creep at 8.2 m
                                    # and the mission carries on from there.
                                    # Reaching the cap means the marker is
                                    # not there, and mark_required decides
                                    # what that means -- by default, land,
                                    # because 9 m is already past it and
                                    # further forward is just further from
                                    # anything known.
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
    BOX_OFFSET_RIGHT = 2.20         # m RIGHT of the marker, onto the boxes.
                                    # MEASURED in the real arena. The scaled
                                    # simulation is a DIFFERENT number
                                    # before scaling -- 1.50 m, which is
                                    # where imav2026_scaled puts the boxes
                                    # relative to platform_1 -- so
                                    # thermal_drop_sitl.launch.py passes its
                                    # own value and does not inherit this
                                    # one. Do not "fix" one to match the
                                    # other; they are two different arenas.
    LAND_SEARCH_DISTANCE = 2.50     # m of forward creep looking for the
                                    # first marker after the sidestep. Short
                                    # on purpose: the sidestep is supposed to
                                    # have landed ON it, so this is an
                                    # acquisition allowance, not a search.
    LAND_SEARCH_SPEED = 0.30

    # ---- the way home -----------------------------------------------------
    #
    #   The marker found after the post-drop sidestep is NOT the landing pad.
    #   It is the pad's opposite number at the far end of the same corridor
    #   the mission flew up, and the landing pad is 8.7 m to 8.8 m BACK along
    #   that corridor. So it is used as a datum: square up on it, then fly
    #   the corridor backwards until the landing marker appears.
    #
    #   Backwards, and not turned round, deliberately. A 180 deg turn at the
    #   end of a mission throws away the one thing the flight has
    #   established -- leg_yaw, the corridor direction measured while
    #   stationary and settled -- and replaces it with a fresh heading
    #   estimate taken mid-rotation, which is the least trustworthy number
    #   EKF2 produces. The nose stays where it was; only the direction of
    #   travel reverses.
    LAND_RETURN_DISTANCE = 9.00     # m of backward creep before giving up.
                                    # The same 8.7-8.8 m plus slack as
                                    # MARK_SEARCH_DISTANCE, and for the same
                                    # reason: it is the same pair of markers.
    LAND_RETURN_SPEED = 0.30        # m/s. As slow as the outbound creep: a
                                    # marker that crosses the frame between
                                    # two detector ticks is a marker that was
                                    # never seen.
    LAND_RETURN_MIN_TRAVEL = 1.00   # m that must be flown before a marker is
                                    # allowed to count. Without it the datum
                                    # marker -- which the aircraft is sitting
                                    # directly over when the leg starts -- is
                                    # instantly "found" again and the
                                    # aircraft lands on the wrong end of the
                                    # arena.

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
    # ------------------------------------------------ following the floor line
    #
    # The arena floor carries a high-contrast pattern in a straight line from
    # the takeoff marker to the one 8.7-8.8 m away. It was laid to give the
    # optical flow something to see; it is also, and more usefully, a corridor
    # drawn on the ground. line_detect reads it off the DOWNWARD camera and
    # publishes the two numbers a straight leg wants: how far off the line the
    # aircraft is, and which way the line runs relative to the nose.
    #
    # It does not replace the latched NED leg -- it CORRECTS it. The leg is
    # still begun from a measured point on a latched bearing, still terminated
    # on distance along that leg, and still protected by the flow-stall guard.
    # When the line is in view the carrot is put on the LINE instead of on the
    # latched bearing; when it is lost the leg carries on exactly as it does
    # today. That is what makes this safe to turn on: the failure mode of the
    # new thing is the old thing.
    LINE_FOLLOW = False         # opt-in. The fixed-bearing leg is the default
                                # and is what has been flown.
    LINE_GAIN = 1.0             # how much of the measured cross-track is taken
                                # out per carrot placement. 1.0 aims the carrot
                                # at the line itself; the leash and the ramp
                                # are what stop that being a lurch.
    LINE_MAX_NUDGE = 0.60       # m. The largest sideways correction a single
                                # frame may ask for. A misdetected line -- a
                                # floor seam, a cable, the edge of a mat --
                                # cannot throw the aircraft further than this.
    LINE_START_AFTER = 2.0      # m of a leg that must be flown before the
                                # strip detector is believed at all. Straight
                                # after takeoff the down camera sees mostly
                                # ArUco MARKER, not floor: a big black-and-
                                # white square whose white cells segment as
                                # "bright thing on dark floor" exactly like the
                                # strip does, and whose axis is whatever the
                                # marker's rotation happens to be. The same
                                # applies leaving the datum marker on the way
                                # home. So the detector is ignored until the
                                # aircraft has flown clear of the pad.
    LINE_LOST_SECONDS = 1.0     # s a line fix stays usable after it arrives.
    LINE_MIN_QUALITY = 0.35     # below this the detector is guessing.
    LINE_MAX_HEADING_DEG = 35.0 # deg. A line further off the nose than this is
                                # not the corridor -- it is a wall join or a
                                # shadow -- and is ignored rather than chased.

    # --------------------------------------------- the end of the corridor
    #
    # The outbound leg has always ended on the marker or on running out of
    # distance. There is a third, better ending: the obstacle at the end of the
    # corridor, seen by the ZED. wall_watch turns its depth image into one
    # number and this is the distance at which that number ends the leg.
    WALL_STOP = False           # opt-in, like the line.
    WALL_STOP_DISTANCE = 1.50   # m. Also enforced by wall_watch itself; this
                                # is the flight node's own copy so that a
                                # mis-parameterised detector cannot fly the
                                # aircraft closer than the mission intends.
    WALL_LOST_SECONDS = 1.0     # s a clearance reading stays usable.

    LEG_STALL_SECONDS = 4.0     # s of unhealthy flow, on a leg, before the
                                # leg is abandoned. See _leg_flow_stalled():
                                # a leg cannot make progress the estimator
                                # is not reporting, and the stage timeout
                                # must not be spent on a stall.
    MARK_STAGE_TIMEOUT = 60.0
    PAD_STAGE_TIMEOUT = 45.0
    ANCHOR_GAIN = 0.35          # how much of each new marker fix goes into
                                # the anchor. Brisk enough to follow a real
                                # correction, slow enough to ignore the
                                # per-frame jitter of a solver working on an
                                # 80-pixel marker.
    MARKER_ANCHOR_MAX_AGE = 3.0 # s an anchor stays usable with no new fix.
                                # Longer than PAD_LOST_SECONDS on purpose:
                                # losing SIGHT of the pad must not lose the
                                # PLACE. Below the handoff height the pad is
                                # out of frame by design.
    GROUND_EFFECT_HEIGHT = 1.20 # m AGL below which the airframe is in its
                                # own downwash, the marker fix gets noisy and
                                # the vehicle wallows. See _handle_land_descend.

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
        self.LAND_RETURN_DISTANCE = float(n('land_return_distance',
                                            self.LAND_RETURN_DISTANCE))
        self.LAND_RETURN_SPEED = float(n('land_return_speed',
                                         self.LAND_RETURN_SPEED))
        self.LAND_RETURN_MIN_TRAVEL = float(n('land_return_min_travel',
                                              self.LAND_RETURN_MIN_TRAVEL))
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
        self.LINE_FOLLOW = bool(self.declare_parameter(
            'line_follow', self.LINE_FOLLOW).value)
        self.LINE_GAIN = float(n('line_gain', self.LINE_GAIN))
        self.LINE_MAX_NUDGE = float(n('line_max_nudge', self.LINE_MAX_NUDGE))
        self.LINE_START_AFTER = float(n('line_start_after',
                                        self.LINE_START_AFTER))
        self.LINE_LOST_SECONDS = float(n('line_lost_seconds',
                                         self.LINE_LOST_SECONDS))
        self.LINE_MIN_QUALITY = float(n('line_min_quality',
                                        self.LINE_MIN_QUALITY))
        self.LINE_MAX_HEADING = math.radians(float(
            n('line_max_heading_deg', self.LINE_MAX_HEADING_DEG)))
        self.WALL_STOP = bool(self.declare_parameter(
            'wall_stop', self.WALL_STOP).value)
        self.WALL_STOP_DISTANCE = float(n('wall_stop_distance',
                                          self.WALL_STOP_DISTANCE))
        self.WALL_LOST_SECONDS = float(n('wall_lost_seconds',
                                         self.WALL_LOST_SECONDS))
        self.LEG_STALL_SECONDS = float(n('leg_stall_seconds',
                                         self.LEG_STALL_SECONDS))
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
        self.ANCHOR_GAIN = float(n('marker_anchor_gain', self.ANCHOR_GAIN))
        self.MARKER_ANCHOR_MAX_AGE = float(n('marker_anchor_max_age',
                                             self.MARKER_ANCHOR_MAX_AGE))
        self.GROUND_EFFECT_HEIGHT = float(n('ground_effect_height',
                                            self.GROUND_EFFECT_HEIGHT))

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
        # true  = the marker found after the sidestep is a DATUM: align on
        #         it, then fly the corridor backwards to the landing pad.
        #         This is the arena as it is actually laid out.
        # false = land on the first marker found after the sidestep, which is
        #         what this file did before the two ends of the corridor were
        #         distinguished. Kept as the escape hatch for a bench or a
        #         one-marker test, where "the pad is the one you can see" is
        #         the whole truth.
        self.LAND_RETURN_ON = bool(self.declare_parameter('land_return', True).value)

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

        self.leg_stall_since = None     # when flow last went unhealthy on a leg
        self.leg_stall_reported = False

        # The floor line, from line_detect. None until one arrives.
        self.line_heading = None        # rad, the line's bearing off the nose
        self.line_cross = None          # m, aircraft to the RIGHT of the line
        self.line_quality = 0.0
        self.line_time = None
        self.line_detected = False
        self.line_using = False         # was the LAST carrot put on the line?
        self.line_had_it = False        # have we EVER had it on this leg?
        # The corridor's end, from wall_watch.
        self.wall_clearance = None      # m, or inf for "clear"
        self.wall_time = None
        self.wall_close = False
        self.outcome_datum = 'marker'   # or 'wall', if the corridor's end won
        self.create_subscription(Vector3Stamped, '/line/track',
                                 self.line_track_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Bool, '/line/detected',
                                 self.line_detected_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Float32, '/wall/clearance',
                                 self.wall_clearance_callback, 10,
                                 callback_group=self.sensor_cbg)
        self.create_subscription(Bool, '/wall/close',
                                 self.wall_close_callback, 10,
                                 callback_group=self.sensor_cbg)

        self.mark_hover_since = None
        self.mark_xy = None             # the marker the boxes are measured off
        self.pad_settle_since = None
        self.pad_lost_since = None
        self.marker_anchor = None       # the pad's position in NED
        self.marker_anchor_time = None
        self.marker_error_log = collections.deque(maxlen=60)
        self._warned_no_attitude = False

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
                + f"cruise at {self.CRUISE_ALTITUDE:.2f} m, "
                + (f"forward up to {self.MARK_SEARCH_DISTANCE:.2f} m at "
                   f"{self.MARK_SEARCH_SPEED:.2f} m/s to the marker, hover "
                   f"{self.MARK_HOVER_SECONDS:.1f} s, "
                   f"{self.BOX_OFFSET_RIGHT:.2f} m RIGHT onto the boxes, then "
                   if self.MARK_SEARCH_ON else "no marker hunt; ")
                + f"a climb to {self.SURVEY_ALTITUDE:.2f} m and the thermal "
                  "survey from there (one observation, no search pattern), "
                  "the drop from "
                + f"{self.DROP_ALTITUDE:.2f} m, the retreat, and "
                + (((f"a datum marker up to {self.LAND_SEARCH_DISTANCE:.2f} m "
                     f"ahead, then up to {self.LAND_RETURN_DISTANCE:.2f} m "
                     f"BACKWARDS at {self.LAND_RETURN_SPEED:.2f} m/s to the "
                     "landing pad, then "
                     if self.LAND_RETURN_ON else "")
                    + f"a precision landing on the marker (centre to "
                      f"{self.PAD_CENTRE_TOLERANCE * 100:.0f} cm, down at "
                      f"{self.PAD_DESCENT_RATE:.2f} m/s, PX4 lands the last "
                      f"{self.PAD_HANDOFF_HEIGHT:.2f} m).")
                   if self.PRECISION_LAND else "a plain landing."))

    # ------------------------------------------------------------- the marker

    def aruco_detected_callback(self, msg):
        self.aruco_flag = bool(msg.data)

    def aruco_point_callback(self, msg):
        """The marker in the CAMERA body frame -> (forward, right, down), m.

        aruco_pose publishes +x right in the image, +y up in the image and +z
        opposite to where the camera looks. With the camera pointing down and
        its image-up towards the nose, that is forward = y, right = x and
        down = -z (down_cam.xacro says the same thing from the other end).

        The DOWN component is kept as well as the other two, which the first
        version of this did not. It is not used as a height -- it carries
        whatever error the quoted field of view has, and the rangefinder
        knows better -- it is used as the third component of a DIRECTION, so
        that the vector can be rotated out of the body frame by the full
        attitude instead of by heading alone. See _aruco_ned().
        """
        self.aruco_seen += 1
        self.aruco_point = (float(msg.point.y), float(msg.point.x),
                            -float(msg.point.z))
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
        """The marker as a (north, east) offset from the aircraft, or None.

        THE FULL ATTITUDE, NOT THE HEADING.

            The body vector is rotated by the attitude quaternion, which is
            what precision_land.py does and for the reason its docstring
            gives: the roll and pitch the airframe is CARRYING IN ORDER TO
            MAKE THE CORRECTION must not themselves show up as extra offset.
            A belly camera on a vehicle tilted by t sees a marker on the
            ground displaced by about h*tan(t); at 2.6 m and 5 deg that is
            23 cm of phantom error, it points the way the vehicle is already
            leaning, and it oscillates with the correction. That is a
            centring loop that hunts -- the "drifting slightly here and
            there" that ground effect makes worse on real hardware, because
            ground effect is exactly what makes the airframe wobble in the
            last metre.

        THE HEADING-ONLY FALLBACK HAD ITS EAST COMPONENT INVERTED.

            It read

                north = f*cos(h) + r*sin(h)
                east  = f*sin(h) - r*cos(h)

            The body axes in NED are forward = (cos h, sin h) and right =
            (-sin h, cos h), so the correct combination is f*forward +
            r*right, i.e. east = f*sin(h) + r*cos(h). The old sign made a
            marker on the RIGHT look like a marker on the LEFT, so the
            aircraft drove away from it at exactly the speed the nudge
            allowed, lost it out of frame a few seconds later, and timed
            out. Measured in the arena: the marker was 0.95 m east, the
            offset came out 0.92 m WEST, and the aircraft ran 3.2 m the
            wrong way before the pad left the camera.

            course_fsm.py's _aruco_ned had the same line, never exercised
            because that simulation runs pad_detector:=false. Both are
            fixed.
        """
        if not self._aruco_fresh():
            return None
        lp = self.local_position
        if lp is None:
            return None
        fwd, right, down = self.aruco_point

        q = self.attitude_q
        if q is not None:
            ned = quat_rotate(q, np.array([fwd, right, down]))
            return np.array([ned[0], ned[1]])

        # No attitude: heading only, uncompensated for tilt, and said out
        # loud because the numbers are quietly worse.
        if not self._warned_no_attitude:
            self._warned_no_attitude = True
            self.get_logger().error(
                "No VehicleAttitude: the marker vector cannot be "
                "tilt-compensated, so every degree the airframe leans adds "
                "about height*tan(lean) of phantom offset. Falling back to a "
                "heading-only rotation. Check vehicle_attitude is in PX4's "
                "DDS topic list.")
        h = lp.heading
        c, sn = math.cos(h), math.sin(h)
        # forward = (cos h, sin h), right = (-sin h, cos h)
        return np.array([fwd * c - right * sn, fwd * sn + right * c])

    def _marker_ned(self):
        """Where the marker IS, in NED, from the newest fix. None if stale."""
        offset = self._aruco_ned()
        if offset is None:
            return None
        lp = self.local_position
        return np.array([lp.x, lp.y]) + offset

    def _update_marker_anchor(self):
        """Keep a smoothed, ABSOLUTE NED position for the marker.

        WHY AN ANCHOR AND NOT JUST THE LIVE OFFSET.

            A relative nudge needs a fix every tick. The last metre of a
            landing is where fixes are scarcest: the pad grows until it
            overflows the frame, the airframe is wallowing in its own ground
            effect, and the detector starts dropping frames exactly when
            precision matters most. A loop with nothing to fall back on
            drifts whenever it blinks.

            So each good fix is turned into an absolute point in NED and
            averaged into an anchor. The aircraft then flies to a PLACE. If
            the marker blinks out the place is still there, and the descent
            carries on instead of stopping or wandering; when the pad
            finally leaves the frame for good, below the handoff height, the
            anchor is what PX4's land is handed.

            The average is exponential and deliberately brisk (ANCHOR_GAIN):
            enough to kill per-frame jitter, not so much that it lags a real
            correction.
        """
        fix = self._marker_ned()
        if fix is None:
            return None
        if self.marker_anchor is None:
            self.marker_anchor = fix
        else:
            g = self.ANCHOR_GAIN
            self.marker_anchor = (1.0 - g) * self.marker_anchor + g * fix
        self.marker_anchor_time = time.monotonic()
        return self.marker_anchor

    def _anchor_error(self):
        """How far the aircraft is from the anchor, or None if it is unusable.

        Unusable means no anchor at all, or one that has had no new fix for
        MARKER_ANCHOR_MAX_AGE. An anchor outlives the SIGHT of the marker on
        purpose -- that is the whole point of it -- but not indefinitely:
        past a few seconds with no fix it is dead reckoning on flow, and
        dead reckoning is not something to land on.
        """
        if self.marker_anchor is None or self.marker_anchor_time is None:
            return None
        if time.monotonic() - self.marker_anchor_time > self.MARKER_ANCHOR_MAX_AGE:
            return None
        lp = self.local_position
        if lp is None:
            return None
        return float(np.linalg.norm(self.marker_anchor - np.array([lp.x, lp.y])))

    def _nudge_onto_marker(self):
        """Fly at the marker ANCHOR, a bounded step at a time.

        Returns the distance still to go, or None when there is no anchor at
        all -- which the callers treat as "stop, do not guess".

        The step is capped at PAD_MAX_NUDGE so that one bad pose cannot throw
        the aircraft further than that, and the commanded point is always
        derived from the ANCHOR rather than from the newest frame, so a
        dropped frame costs nothing.
        """
        self._update_marker_anchor()
        error = self._anchor_error()
        if error is None:
            return None
        lp = self.local_position
        here = np.array([lp.x, lp.y])
        step = (self.marker_anchor - here) * self.PAD_GAIN
        n = float(np.linalg.norm(step))
        if n > self.PAD_MAX_NUDGE:
            step = step / n * self.PAD_MAX_NUDGE
        self._move_to(here + step)
        self._check_runaway(error)
        return error

    def _check_runaway(self, error):
        """Shout if the centring is making things worse, instead of leaving.

        A centring loop that is wired up backwards does not fail, it
        DIVERGES: it drives away from the marker at exactly the speed the
        nudge allows, loses it out of frame, and reports a timeout somewhere
        far from the pad. That is what a mirrored camera mount, a wrong
        cam_yaw_deg or an inverted axis looks like from the log, and it cost
        a full flight to find. So the error is watched, and if it is bigger
        than it was a second ago, several times running, the stage says so in
        as many words rather than flying on.
        """
        now = time.monotonic()
        self.marker_error_log.append((now, error))
        recent = [(t, e) for t, e in self.marker_error_log if now - t <= 2.0]
        if len(recent) < 8 or recent[-1][1] < self.PAD_CENTRE_TOLERANCE:
            return
        if recent[-1][1] <= recent[0][1] + 0.05:
            return
        self.get_logger().error(
            f"CENTRING IS DIVERGING: {recent[0][1]:.2f} m -> "
            f"{recent[-1][1]:.2f} m in {now - recent[0][0]:.1f} s. The marker "
            "offset is being resolved into the wrong direction -- check the "
            "camera's mounting (flip_lr / cam_yaw_deg) and that "
            "/aruco/point's forward and right match where the marker really "
            "is. Holding rather than chasing it further.",
            throttle_duration_sec=5.0)
        self.moving = False

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
        self.leg_stall_since = None
        self.leg_stall_reported = False
        self.line_using = False
        self.line_had_it = False
        self.MOVE_SPEED = speed

    def _follow_leg(self, allow_line=False):
        """Put the carrot ON the line, a lookahead ahead of us, every tick.

        With allow_line and a usable fix from line_detect, the carrot goes on
        the line PAINTED ON THE FLOOR instead of the one latched in NED at the
        start of the leg. The returned (along, cross) are still measured
        against the latched leg either way, so the distance accounting, the
        timeout and the cross-track report do not change meaning depending on
        what the camera can see -- only the steering does.

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

        # The painted line wins over the latched bearing whenever it is
        # trustworthy -- it is the corridor itself, where the latched bearing
        # is only ever a measurement of where the corridor was thought to be.
        using_line = False
        if allow_line and along < self.LINE_START_AFTER:
            # Still over the pad we left. See LINE_START_AFTER.
            self.get_logger().info(
                f"LINE: held off for the first {self.LINE_START_AFTER:.2f} m "
                f"({along:.2f} m so far) -- the camera is still over the "
                "marker we took off from, not the strip.",
                throttle_duration_sec=2.0)
        elif allow_line and self._line_is_usable():
            line_carrot = self._line_carrot()
            if line_carrot is not None:
                # Never let the line push the carrot PAST the end of the leg.
                # The distance cap is the mission's, not the paint's.
                overshoot = float((line_carrot - self.leg_start) @ u) - self.leg_length
                if overshoot > 0.0:
                    line_carrot = line_carrot - u * overshoot
                carrot = line_carrot
                using_line = True
        if using_line != self.line_using:
            if using_line:
                self.line_had_it = True
                self.get_logger().warning(
                    f"LINE: steering on the floor pattern "
                    f"({math.degrees(self.line_heading):+.1f} deg off the nose, "
                    f"{self.line_cross:+.2f} m off it, quality "
                    f"{self.line_quality:.2f}).")
            else:
                self.get_logger().warning(
                    "LINE: lost it. Back on the latched bearing at "
                    f"{math.degrees(self.leg_heading):+.2f} deg"
                    + (" -- which is what this leg flew before line_follow "
                       "existed, so this is a fallback and not a failure."
                       if self.line_had_it else "."))
            self.line_using = using_line

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

    # ------------------------------------------------------ the floor line

    def line_track_callback(self, msg):
        """x = heading error (rad), y = cross-track (m, NaN if unscaled)."""
        self.line_heading = float(msg.vector.x)
        cross = float(msg.vector.y)
        # NaN means line_detect had no height and so could not put the
        # cross-track in metres. The heading half is still good.
        self.line_cross = None if math.isnan(cross) else cross
        self.line_quality = float(msg.vector.z)
        self.line_time = time.monotonic()

    def line_detected_callback(self, msg):
        self.line_detected = bool(msg.data)

    def wall_clearance_callback(self, msg):
        value = float(msg.data)
        # NaN is "I cannot tell", which is NOT "clear". Drop it rather than
        # let it read as a corridor that goes on for ever.
        if math.isnan(value):
            return
        self.wall_clearance = value
        self.wall_time = time.monotonic()

    def wall_close_callback(self, msg):
        self.wall_close = bool(msg.data)

    def _line_is_usable(self):
        """Is there a line fix good enough to steer on RIGHT NOW?

        Every one of these is a way a floor pattern detector can be confidently
        wrong, and a wrong line steers the aircraft into the wall it was meant
        to avoid:

          marker in view    THE IMPORTANT ONE. The strip detector segments
                            "bright thing on dark floor", and an ArUco marker
                            is a bright thing on a dark floor. Over the pad it
                            will happily report the MARKER's axis as the
                            corridor, which is whatever angle the marker was
                            laid at. While /aruco/detected is true the down
                            camera is looking at a marker, so the strip answer
                            is not to be trusted -- and the mission does not
                            need it then anyway, because a marker in view is
                            about to end this leg.
          not detected      the detector's own debounce has not settled
          stale             the frames stopped; the last answer is not news
          low quality       few segments voted for the winning orientation
          far off the nose  a line more than line_max_heading_deg off the nose
                            is a wall join, a shadow or the edge of a mat --
                            the corridor is, by construction, roughly ahead
          no cross-track    no rangefinder, so the offset has no scale
        """
        if not (self.LINE_FOLLOW and self.line_detected):
            return False
        if self._marker_is_under_us():
            return False
        if self.line_time is None:
            return False
        if time.monotonic() - self.line_time > self.LINE_LOST_SECONDS:
            return False
        if self.line_quality < self.LINE_MIN_QUALITY:
            return False
        if self.line_heading is None or self.line_cross is None:
            return False
        return abs(self.line_heading) <= self.LINE_MAX_HEADING

    def _line_carrot(self):
        """Where to fly to, put on the LINE rather than on the latched bearing.

        The line's bearing in NED is the aircraft's heading plus the detector's
        heading error, so this needs no survey of the hall and no leg_bearing
        argument -- the corridor is wherever the paint says it is.

        The carrot goes LEG_LOOKAHEAD along that bearing and LINE_GAIN of the
        measured cross-track back towards the line, clamped to LINE_MAX_NUDGE.
        The clamp is the whole safety argument: one bad frame can bend the
        course by at most that much, and the next good frame undoes it.

        WHICH WAY ALONG THE LINE is not the camera's to say. A painted line is
        undirected -- line_detect reports its bearing mod pi -- and LAND_RETURN
        flies this same corridor BACKWARDS, nose still pointing the way it came
        from. So the sense is taken from the LEG, not from the nose: of the two
        opposite directions the paint allows, take the one that goes the way
        this leg is travelling. Get this wrong and the return leg drives
        forwards up the corridor it has just come down.
        """
        lp = self.local_position
        if lp is None:
            return None
        bearing = wrap_pi(lp.heading + self.line_heading)
        u = np.array([math.cos(bearing), math.sin(bearing)])
        if u @ self.leg_unit < 0.0:
            u = -u
        right = np.array([-u[1], u[0]])
        nudge = max(-self.LINE_MAX_NUDGE,
                    min(self.LINE_MAX_NUDGE, self.line_cross * self.LINE_GAIN))
        here = np.array([lp.x, lp.y])
        # cross is positive when the aircraft is RIGHT of the line, so the
        # correction goes LEFT, which is minus the right-hand unit vector.
        return here + u * self.LEG_LOOKAHEAD - right * nudge

    def _leg_flow_stalled(self):
        """True once a leg has been unable to make progress long enough to give up.

        Every leg ends on `along`, which is the ESTIMATE projected onto the
        line -- and the carrot is leashed to the estimate as well, in
        _step_xy_ramp(). So when the flow stops correcting, both halves fail
        at the same instant and in the same direction: the commanded point is
        pinned to a position that is no longer advancing, PX4 sees no error,
        the aircraft stops dead in mid-leg, and `along` freezes with it. The
        one thing that keeps moving is the stage clock, which then lands the
        aircraft where it stopped and reports "no marker found" -- a leg that
        was never flown, blamed on a marker that was never passed.

        Measured on the real aircraft: the creep halted around 4 m of a 9 m
        leg and sat there until the 60 s timeout put it down.

        So while the estimate is stalled the stage clock is HELD. A stall that
        clears costs the leg nothing but the seconds it lasted; one that does
        not clear is reported as a lost estimate rather than as a missing
        marker, because those two have opposite fixes.

        Deliberately uses flow_is_healthy() and not xy_valid: EKF2 keeps
        xy_valid true while it coasts on the IMU, which is exactly the case
        this has to catch.
        """
        now = time.monotonic()
        if self.flow_is_healthy():
            if self.leg_stall_since is not None:
                if self.leg_stall_reported:
                    self.get_logger().warning(
                        f"LEG: flow healthy again after "
                        f"{now - self.leg_stall_since:.1f} s. Carrying on; the "
                        "stage clock was held for the whole stall.")
                self.leg_stall_since = None
                self.leg_stall_reported = False
            return False

        if self.leg_stall_since is None:
            self.leg_stall_since = now
        stalled_for = now - self.leg_stall_since
        # The timeout exists to catch a leg that ran out of arena, not one
        # that ran out of estimator. Hold it while we are not moving.
        self._restart_stage_clock()
        lp = self.local_position
        self.get_logger().error(
            f"LEG STALLED {stalled_for:.1f}/{self.LEG_STALL_SECONDS:.1f} s: "
            "optical flow is not correcting, so the commanded point is "
            "leashed to a position that is not advancing and the aircraft is "
            "not going anywhere. "
            + (f"xy_valid={lp.xy_valid} v_xy_valid={lp.v_xy_valid} "
               f"dist_bottom={lp.dist_bottom:.2f} m "
               f"(need > {self.FLOW_MIN_AGL:.2f} m) "
               f"rng_ok={self.rangefinder_is_healthy()}"
               if lp is not None else "no local position at all")
            + ". Stage clock HELD.",
            throttle_duration_sec=1.0)
        if stalled_for > self.LEG_STALL_SECONDS:
            self.leg_stall_reported = True
            return True
        self.leg_stall_reported = True
        return False

    def _abandon_leg_for_flow(self, along):
        """End the mission on a lost estimate, saying so.

        Landing is the only honest ending: without a lateral estimate there
        is no flying the rest of the leg and no knowing where the aircraft
        would be flying to. It goes down where it is, which is where it has
        been sitting anyway.
        """
        why = (f"optical flow unhealthy for more than "
               f"{self.LEG_STALL_SECONDS:.1f} s at {along:.2f} m along the leg")
        self.outcome = f"ENDED: {why}; the leg could not be flown."
        self.get_logger().error(
            f"{self.current_stage}: {why}. This is NOT a missing marker -- the "
            "aircraft never covered the ground. Landing here. Check the ARK "
            "Flow: surface texture and lighting under the corridor, "
            f"dist_bottom against flow_min_agl ({self.FLOW_MIN_AGL:.2f} m), "
            "and optical_flow.quality in the PX4 log.")
        self._finish(why, land=True)

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
            self.LAND_ALIGN: self._handle_land_align,
            self.LAND_RETURN: self._handle_land_return,
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
        # The CRUISE height, not the survey height. This stage is looking for
        # a marker on the floor, and low is better for that: more pixels on
        # the marker, a smaller footprint so "in frame" means "nearly
        # underneath", and the cone is carried across the arena at knee
        # height rather than head height. The climb is paid for once, over
        # the boxes, where the aircraft is stationary anyway -- see
        # cruise_altitude in thermal_drop.py.
        self._set_altitude(self.CRUISE_ALTITUDE)
        self._begin_leg(self.leg_yaw, self.MARK_SEARCH_DISTANCE,
                        self.MARK_SEARCH_SPEED)
        self.get_logger().warning(
            f"MARK_SEARCH: forward at {self.MARK_SEARCH_SPEED:.2f} m/s, up to "
            f"{self.MARK_SEARCH_DISTANCE:.2f} m, at "
            f"{self.CRUISE_ALTITUDE:.2f} m, on a LINE at "
            f"{math.degrees(self.leg_heading):+.2f} deg held to "
            f"{self.LEG_LOOKAHEAD:.2f} m lookahead, until the downward camera "
            f"puts a marker under us. The first {self.MARK_MIN_TRAVEL:.2f} m "
            "do not count -- that is the pad we armed on.")

    def _handle_mark_search(self):
        along, cross = self._follow_leg(allow_line=True)
        # Before anything is concluded from `along`: is `along` still moving?
        if self._leg_flow_stalled():
            self._abandon_leg_for_flow(along)
            return
        if self._marker_is_under_us() and along >= self.MARK_MIN_TRAVEL:
            self._begin_mark_hover(along)
            return
        # The corridor ends at the obstacle whether or not a marker was ever
        # seen. Checked AFTER the marker, so a marker sitting at the end of the
        # run still wins and the boxes are still measured off it.
        if self._wall_is_close(along):
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

    def _wall_is_close(self, along):
        """Has the corridor ended in front of us? If so, end the leg HERE.

        Treated as ARRIVAL, not as failure. The obstacle at the end of the run
        is at a known place relative to the boxes, so stopping wall_stop_distance
        short of it is a survey point in the same way the marker is -- and the
        sidestep that follows is measured from wherever the aircraft stopped.

        THAT IS THE ASSUMPTION, AND IT IS WORTH SAYING OUT LOUD: box_offset_right
        was MEASURED from the marker, not from the wall. Ending here substitutes
        one datum for the other and inherits whatever the difference between
        them is. It is the right ending when the marker was missed and the run
        would otherwise be flown into the obstacle; it is not as good as the
        marker, and the log says which one the mission got.

        Checked only while the clearance is FRESH. A dead depth topic must not
        read as a corridor with no end -- but neither may it stop the mission,
        so a stale reading simply does not fire this.
        """
        if not self.WALL_STOP:
            return False
        if self.wall_time is None:
            return False
        if time.monotonic() - self.wall_time > self.WALL_LOST_SECONDS:
            self.get_logger().error(
                "WALL: /wall/clearance has gone stale. Not stopping on it -- "
                "the distance cap and the timeout are the only things left "
                "ending this leg. Is wall_watch running?",
                throttle_duration_sec=5.0)
            return False
        clearance = self.wall_clearance
        if clearance is None or not (self.wall_close
                                     or clearance <= self.WALL_STOP_DISTANCE):
            return False
        self.outcome_datum = 'wall'
        self.get_logger().warning(
            f"MARK_SEARCH: obstacle {clearance:.2f} m ahead, at or inside the "
            f"{self.WALL_STOP_DISTANCE:.2f} m stop distance, after {along:.2f} m. "
            "That is the end of the corridor. Treating it as ARRIVAL and "
            "stepping across to the boxes from HERE -- note the sidestep is "
            "now measured off the WALL and not off the marker, which is a "
            "different datum. " + self._leg_report("track") + ".")
        self.moving = False
        self._begin_box_offset()
        return True

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
        # Still the cruise height. The climb belongs to the SURVEY, which
        # starts where this leg ends: climbing before the sidestep would fly
        # the sidestep high for no gain, and the survey has to wait for the
        # climb either way.
        self._set_altitude(self.CRUISE_ALTITUDE)
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
            f"{math.degrees(self.leg_heading):+.2f} deg, at "
            f"{self.CRUISE_ALTITUDE:.2f} m. The thermal survey starts there, "
            f"with the climb to {self.SURVEY_ALTITUDE:.2f} m.")

    def _handle_box_offset(self):
        along, cross = self._follow_leg()
        # A stall here is worse than it looks: the timeout below would survey
        # "over the boxes" from wherever the sidestep died, which on a 2.20 m
        # step is bare floor beside them.
        if self._leg_flow_stalled():
            self._abandon_leg_for_flow(along)
            return
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
        # NO line following on this leg, deliberately. It runs from the
        # RETREAT point -- two sidesteps off the corridor, over the boxes --
        # and is hunting the datum marker, so the strip is not reliably
        # underneath and the marker it is looking for is exactly the thing the
        # strip detector mistakes for a strip. Line following resumes on
        # LAND_RETURN, once that marker has been found and squared up on,
        # which is where the aircraft is back on the corridor.
        gone, cross = self._follow_leg(allow_line=False)
        if self._leg_flow_stalled():
            self._abandon_leg_for_flow(gone)
            return
        if self._marker_is_under_us():
            if self.LAND_RETURN_ON:
                self._begin_land_align()
            else:
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

    # ------------------------------------------------- LAND_ALIGN / RETURN

    def _begin_land_align(self):
        """Square up on the datum marker before the run home.

        Centring here is not cosmetic. Everything that follows is dead
        reckoning on flow along a 9 m line, and a line has two errors: where
        it starts and which way it points. This stage fixes the first. Half a
        metre of offset left here is half a metre of offset carried the whole
        way back, and the landing marker is only found at all because it
        passes under a camera whose footprint at this height is a couple of
        metres wide.

        It reuses the pad centring loop -- anchor, bounded nudge, divergence
        watchdog -- because "get over that marker accurately" is the same
        problem as it is at the landing pad. What it does NOT do is descend.
        """
        self.MOVE_SPEED = self.LAND_RETURN_SPEED
        self.pad_settle_since = None
        self.marker_anchor = None       # the datum, not the box we dropped on
        self.marker_error_log.clear()
        self._enter_stage(self.LAND_ALIGN)
        self._nudge_onto_marker()
        self.get_logger().warning(
            "LAND_ALIGN: marker"
            + (f" (id {self.aruco_id})" if self.aruco_id else "")
            + " found after the sidestep. This is the DATUM, not the landing "
            f"pad: centring to {self.PAD_CENTRE_TOLERANCE * 100:.0f} cm, then "
            f"flying back down the corridor up to "
            f"{self.LAND_RETURN_DISTANCE:.2f} m to the pad.")

    def _handle_land_align(self):
        error = self._nudge_onto_marker()
        now = time.monotonic()
        if error is None:
            # No anchor. Do not start the run home from a guess -- but do not
            # sit here for ever either: the corridor direction is known
            # independently of the marker, so a lost datum costs accuracy,
            # not the mission.
            self.moving = False
            if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
                self.get_logger().warning(
                    "LAND_ALIGN: datum marker gone and not coming back. "
                    "Starting the run home from here, on the last fix.")
                self._begin_land_return()
            return
        if error <= self.PAD_CENTRE_TOLERANCE:
            if self.pad_settle_since is None:
                self.pad_settle_since = now
            elif now - self.pad_settle_since >= self.PAD_CENTRE_SECONDS:
                self._begin_land_return()
            return
        self.pad_settle_since = None
        if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
            self.get_logger().warning(
                f"LAND_ALIGN: still {error:.2f} m off after "
                f"{self.PAD_STAGE_TIMEOUT:.0f} s. Going home anyway -- that "
                "offset is carried the whole way back.")
            self._begin_land_return()
            return
        self.get_logger().info(f"LAND_ALIGN: {error:.2f} m off the datum.",
                               throttle_duration_sec=1.0)

    def _begin_land_return(self):
        """Fly the corridor BACKWARDS, on the heading it was measured on.

        leg_yaw + 180 deg, so the LINE reverses while _follow_leg keeps
        pointing the nose at leg_yaw -- see the note in _follow_leg about why
        the nose is aimed at the corridor and not at the direction of travel.
        The aircraft flies backwards over the ground, which the flow sensor
        and the downward camera do not care about, and no 180 deg turn is
        made at the one point in the flight where the heading estimate is
        worth the most.
        """
        self.pad_settle_since = None
        self.marker_anchor = None       # the datum is behind us now
        self.marker_error_log.clear()
        self._enter_stage(self.LAND_RETURN)
        self._begin_leg(wrap_pi(self.leg_yaw + math.pi),
                        self.LAND_RETURN_DISTANCE, self.LAND_RETURN_SPEED)
        self.get_logger().warning(
            f"LAND_RETURN: backwards along the corridor at "
            f"{self.LAND_RETURN_SPEED:.2f} m/s, up to "
            f"{self.LAND_RETURN_DISTANCE:.2f} m, on a line at "
            f"{math.degrees(self.leg_heading):+.2f} deg (nose still at "
            f"{math.degrees(self.leg_yaw):+.2f} deg), until the landing "
            f"marker is under us. The first {self.LAND_RETURN_MIN_TRAVEL:.2f} m "
            "do not count -- that is the datum we just left.")

    def _handle_land_return(self):
        # The line is followed here too, and this leg is flown BACKWARDS down
        # the same corridor. A nadir detector does not care: the painted line
        # is under the aircraft either way, and _line_carrot() takes its
        # bearing from the CURRENT heading, which _follow_leg holds on
        # leg_yaw throughout. So no 180 deg turn is needed to use the line on
        # the way home -- see the note in the module docstring.
        along, cross = self._follow_leg(allow_line=True)
        if self._leg_flow_stalled():
            self._abandon_leg_for_flow(along)
            return
        if self._marker_is_under_us() and along >= self.LAND_RETURN_MIN_TRAVEL:
            self.get_logger().warning(
                f"LAND_RETURN: landing marker after {along:.2f} m, "
                + self._leg_report("track") + ".")
            self._begin_land_centre()
            return
        timed_out = self._in_stage_for() > self.MARK_STAGE_TIMEOUT
        if along >= self.LAND_RETURN_DISTANCE - self.MOVE_TOLERANCE or timed_out:
            # NOT a failure, and not a reason to keep looking. The cone is in
            # the box -- the mission is scored. 9 m is already past where the
            # pad is, so the aircraft is now the far side of it, and the
            # honest ending is to put it down here rather than wander an
            # arena it has lost its datum in.
            self.outcome = ("MISSION COMPLETE: cone dropped on the hottest "
                            f"box. No landing marker in {along:.2f} m of the "
                            f"run home ({self.aruco_seen} poses seen); landed "
                            "off-pad.")
            self.get_logger().warning(
                f"LAND_RETURN: no marker in {along:.2f} m"
                f"{' (timed out)' if timed_out else ''}. "
                + self._leg_report("track")
                + ". Going into the landing sequence here.")
            self._finish("run home complete, no marker to land on", land=True)
            return
        self.get_logger().info(
            f"LAND_RETURN: {along:.2f}/{self.LAND_RETURN_DISTANCE:.2f} m back, "
            f"{cross:+.2f} m off the line, "
            f"marker {'YES' if self.aruco_flag else 'no'} "
            f"({self.aruco_seen} poses seen).",
            throttle_duration_sec=1.0)

    def _begin_land_centre(self):
        self.MOVE_SPEED = self.LAND_SEARCH_SPEED
        self.pad_settle_since = None
        self.marker_anchor = None       # a fresh pad, not the one we dropped on
        self.marker_error_log.clear()
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
        """Down onto the anchor, holding station on it the whole way.

        HOW THIS DIFFERS FROM "STOP IF THE MARKER BLINKS"

            The first version stopped the descent whenever a frame was
            missed. That is the wrong instinct for the last two metres,
            which is precisely where frames go missing: the pad is growing
            towards the edges of the image, the airframe is wallowing in its
            own ground effect, and the solver is working on a marker that
            keeps leaving the frame. Stopping there leaves the aircraft
            hovering IN ground effect, which is the least stable place it
            can be, waiting for a fix that gets less likely the longer it
            waits.

            So the descent flies to the ANCHOR -- an absolute NED point,
            built from the fixes taken while the marker WAS visible -- and
            keeps going through a blink. Only when the anchor itself goes
            stale (marker_anchor_max_age with no new fix at all) does it
            stop and hold.

        GROUND EFFECT

            Below GROUND_EFFECT_HEIGHT the downwash comes back off the pad,
            the airframe wanders, and every metre of that wander is a metre
            of marker error that is NOT a real position error. Two things
            change there: the descent slows to half rate, so there is more
            time for the correction to work than for the wobble to build,
            and the gate on "are we centred enough to keep going down" is
            relaxed rather than tightened -- fighting a wobble by chasing it
            is what makes the aircraft hunt. It is the anchor that holds the
            position, not the last frame.
        """
        now = time.monotonic()
        alt = self.relative_altitude()
        error = self._nudge_onto_marker()
        in_ground_effect = alt is not None and alt <= self.GROUND_EFFECT_HEIGHT

        if error is None:
            # No anchor at all, or it has gone stale. Hold, do not guess.
            self.moving = False
            if self.pad_lost_since is None:
                self.pad_lost_since = now
                self.get_logger().error(
                    "LAND_DESCEND: no marker and no usable anchor. Holding "
                    "altitude until one of them is back.")
            if now - self.pad_lost_since > self.PAD_STAGE_TIMEOUT:
                self.get_logger().error(
                    "LAND_DESCEND: nothing came back. Landing from here.")
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

        # Only go down while we are actually over the pad. Out past the
        # gate, hold the height and let the correction catch up -- descending
        # while 40 cm off just arrives 40 cm off, lower down, with less room
        # to fix it.
        gate = self.PAD_CENTRE_TOLERANCE * (2.0 if in_ground_effect else 1.5)
        if error > gate:
            self.get_logger().info(
                f"LAND_DESCEND: holding {alt:.2f} m, {error:.2f} m off "
                f"(want {gate:.2f} m before going lower).",
                throttle_duration_sec=1.0)
            return

        rate = self.PAD_DESCENT_RATE * (0.5 if in_ground_effect else 1.0)
        # DROP_ALTITUDE is the floor _set_altitude() clamps to, and it is
        # above the handoff height, so the descent is walked directly here.
        self.commanded_altitude = max(
            self.PAD_HANDOFF_HEIGHT,
            self.commanded_altitude - rate * self.TIMER_PERIOD)
        self.target_z = self.home_z - self.commanded_altitude
        self.get_logger().info(
            f"LAND_DESCEND: alt {'n/a' if alt is None else f'{alt:.2f}'} -> "
            f"{self.commanded_altitude:.2f} m at {rate:.2f} m/s"
            + (" (ground effect)" if in_ground_effect else "")
            + f", {error:.2f} m off the marker.",
            throttle_duration_sec=1.0)

    # ---------------------------------------------------------------- status

    def publish_status(self):
        if self.current_stage not in self.FSM_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.current_stage in (self.MARK_SEARCH, self.LAND_SEARCH,
                                  self.BOX_OFFSET, self.LAND_RETURN):
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
            err = self._anchor_error()
            detail = 'mk?' if err is None else f"mk{err:.2f}"
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
        if lp is not None:
            pivot = np.array([lp.x, lp.y])
            c, sn = math.cos(delta), math.sin(delta)
            rot = np.array([[c, -sn], [sn, c]])
            if self.mark_xy is not None:
                self.mark_xy = pivot + rot @ (self.mark_xy - pivot)
            # The pad has not moved; the frame around it has.
            if self.marker_anchor is not None:
                self.marker_anchor = pivot + rot @ (self.marker_anchor - pivot)
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
