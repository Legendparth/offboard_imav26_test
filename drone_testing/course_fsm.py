"""
The obstacle course as one flight: WINDOW -> OVER the red bar -> UNDER the
blue bar -> land. ARK Flow localisation, ZED as a camera only.

    arm -> climb -> hold -> [the window mission, unchanged] -> HANDOFF ->
    RED_RISE -> RED_CROSS -> BLUE_DROP -> BLUE_CROSS -> land

    q -> abort into a controlled descent.   k -> force-disarm.

    With window_after_tubes:=true the window mission is flown a SECOND time
    once the tubes finish:

        TUBE_EXIT -> WINDOW2_RISE -> WINDOW2_HOLD -> [the window mission] ->
        the landing the tubes would have ended with

    WINDOW2_RISE climbs straight up, where it stands, to window2_altitude
    (takeoff_altitude by default) and WINDOW2_HOLD stands still there for
    window2_hold_seconds before a frame is measured. That run-up is not
    decoration: the tubes end at the gap altitude, UNDER the cross tube, and
    ALIGN flies to a standoff point at the WINDOW's height -- so starting the
    search from down there turns the approach into a long diagonal climb back
    through the obstacle the aircraft has just squeezed under. Getting the
    height back first, on the spot, makes the second pass the same flight the
    first one was.

    The traverse itself is the same SCAN/LOCK/AIM/ALIGN/TRAVERSE code, with
    the detector switched back on and the estimator emptied of the first
    window, and only then does the pad landing that would have followed the
    tubes. Nothing of the course geometry is applied to that pass: it is flown
    entirely on what the camera measures, so there has to BE a window past the
    gate.

WHY THIS IS A SUBCLASS OF WindowTraverse
    The window traversal has been flown and it works. Everything up to and
    including "the aircraft is through the window" is that exact code -- not a
    copy of it, the class itself. This node only replaces what happens after
    the traverse: instead of holding and landing, it hands off to the bars.
    A fix to the window mission is therefore automatically a fix here, and
    nothing about the part that works has been re-typed.

THE GEOMETRY
    Everything is measured along ONE line: the window's committed traverse
    line, which is the window centreline the aircraft was just aligned on to
    within centimetres. The bars are assumed to sit square across that same
    line, which is how the course is laid out.

        window        red bar        blue bar
          |    1.0 m    |    1.0 m     |   blue_exit   land
          |------P0-----|------P1------|-------P2
               0.5 m         0.5 m

    P0 and P1 are the MIDPOINTS of the two gaps, and every vertical move in
    the course happens at a midpoint and only at a midpoint. With 1.0 m gaps
    that is 0.5 m to the obstacle on either side, which on a 0.26 m airframe
    leaves 0.37 m from a prop tip to the nearest thing it could hit -- the
    most room there is anywhere in the course, and the reason nothing climbs
    or descends anywhere else. The window mission's exit_distance is therefore
    forced to half the window-to-red gap, so the traverse itself ends on P0.

    Translations happen only once the aircraft is at the altitude for the
    obstacle ahead: the crossing stages are not entered until the climb or the
    descent has settled, so the airframe is never moving towards a bar at a
    height that would hit it.

THE ALTITUDES
    Solved for the airframe, not the origin, as bar_cross does:

        red   (over)   red_bar_height + bar_radius + red_clearance + body_below
        blue  (under)  blue_bar_height - bar_radius - blue_clearance - body_above

    which for 1980 / 800 mm, a 20 mm radius, 25 / 20 cm of clearance and the
    standard mounting is 2.41 m and 0.48 m. Both are checked against
    min_altitude / max_altitude, and the blue one against the optical-flow
    floor, at START-UP -- a course that cannot be flown is refused on the
    ground, not discovered over a bar.

WHY THE BARS ARE FLOWN ON KNOWN GEOMETRY, NOT MEASURED
    The course leaves 0.5 m between the aircraft and each bar at the point it
    would have to look at it. From there the camera cannot do the job:

      * a 1.5 m bar at 0.5 m subtends 112 degrees; the ZED sees 90. Both ends
        are always off the edges of the frame, so its length -- and the check
        that the crossing is not near an end -- cannot be measured;
      * 0.5 m is at the near limit of the ZED's stereo depth, which is the
        number every height in bar_cross is built from;
      * the red bar is only in frame from 1.7-2.3 m altitude, and the blue
        bar is invisible from the red bar's crossing altitude entirely (it is
        73 degrees below the optical axis).

    And the things a measurement would provide are already known: the heights
    are the rules settings you chose, and the positions come from a window the
    aircraft has just measured and aligned to. So the bars are flown from
    those numbers. bar_cross.py keeps its measured mode for standalone testing
    at a distance where it can actually see.

WHY IT DOES NOT GO BACK TO 1.2 m BETWEEN OBSTACLES
    Every obstacle dictates its own altitude, so a "home" height between them
    is only somewhere to pass through. Returning to 1.2 m after the red bar
    costs an extra 1.2 m descent and 0.7 m climb-then-descent for nothing,
    and in a 1 m gap it means more time moving vertically next to a bar, not
    less. Each vertical move here goes directly from one obstacle's altitude
    to the next.

WHEN OPTICAL FLOW DROPS OUT
    What is safe depends entirely on where the aircraft is, so it is decided
    per stage rather than once:

      TRAVERSE    the window's blind push is capped by DISTANCE to P0, not
                  just by time -- the inherited 3 s would carry the aircraft
                  1.35 m, straight into the red bar. It stops at P0 and lands.
      RED_RISE /  hold still at the midpoint, and land if flow is not back in
      BLUE_DROP   course_flow_timeout: a straight-down descent at a midpoint
                  passes nothing.
      RED_CROSS   the worst place to lose it -- above a bar, where descending
                  lands on it. Push on open-loop for exactly the distance that
                  was left to P1, then land there, past the bar.
      BLUE_CROSS  land immediately: under or past the bar, straight down is
                  away from it.
THE FAILSAFES, AND WHAT EACH ONE IS FOR
    Every place this mission depends on a CAMERA finding something now has an
    answer for "it did not", and none of those answers is an immediate
    landing:

      the tube gate    the look-from back-off is capped (tube_back_off_max) so
                       the aircraft does not reverse over the blue bar it has
                       just flown under; a sweep that sees nothing retries at
                       tube_scan_alt_steps different heights; and if nothing
                       ever solves, the gate is crossed BLIND on its known
                       geometry -- half a spacing to the tube_gap_prefer side
                       of the track, at the template altitude (tube_blind).

      the 2nd window   looked for window2_search_timeout seconds, and if it
                       never appears the aircraft goes OVER it instead: up to
                       the red bar's altitude, across the distance the
                       traverse would have covered, and back down
                       (window2_skip). Every other way the window mission can
                       fail routes here too, through _abandon().

      the rangefinder  an outage holds position -- dead still, height on the
                       barometer -- and resumes only once the fusion has been
                       healthy CONTINUOUSLY for course_hold_confirm_seconds.
                       If it never comes back, course_hold_press_on carries
                       the course on rather than landing, provided the EKF
                       still has a height and the flow still holds position.
                       Reproduce the whole thing in SITL with
                       rng_dropout:=true.

      the climb        is in offboard_sequence, not here: a takeoff that does
                       not settle is now diagnosed before it is abandoned.
                       See _takeoff_timed_out().

    The first window has no failsafe and is not given one: the entire course
    is laid out from where that window is measured, so there is nothing to
    fall back ON.
"""

import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.offboard_sequence import nav_state_name, spin_node, wrap_pi
from drone_testing.tube_cross import (TubeEstimator, front_plane,
                                      solve_from_hole, solve_gap)
from drone_testing.tube_detect import HOLE_STRIDE, STRIDE
from drone_testing.window_traverse import WindowTraverse


class CourseFSM(WindowTraverse):

    RED_RISE = "RED_RISE"
    RED_CROSS = "RED_CROSS"
    BLUE_DROP = "BLUE_DROP"
    BLUE_CROSS = "BLUE_CROSS"

    TUBE_CLIMB = "TUBE_CLIMB"
    TUBE_APPROACH = "TUBE_APPROACH"  # to the look-from point, never past it
    TUBE_SCAN = "TUBE_SCAN"         # stand still, sweep the yaw, look at it
    TUBE_SEARCH = "TUBE_SEARCH"
    TUBE_LOCK = "TUBE_LOCK"
    TUBE_ALIGN = "TUBE_ALIGN"
    TUBE_PASS = "TUBE_PASS"
    TUBE_SHIFT = "TUBE_SHIFT"
    TUBE_EXIT = "TUBE_EXIT"

    WINDOW2_RISE = "WINDOW2_RISE"   # back up to the search altitude
    WINDOW2_HOLD = "WINDOW2_HOLD"   # stand still there before looking
    # The skip: over the window's wall rather than through the window.
    WINDOW2_SKIP_RISE = "WINDOW2_SKIP_RISE"
    WINDOW2_SKIP_CROSS = "WINDOW2_SKIP_CROSS"
    WINDOW2_SKIP_DROP = "WINDOW2_SKIP_DROP"

    START_OFFSET = "START_OFFSET"   # step right off the pad, before the sweep

    PAD_OFFSET = "PAD_OFFSET"       # step right, clear of the tube line
    PAD_SEARCH = "PAD_SEARCH"       # creep forward until the marker is seen
    PAD_CENTRE = "PAD_CENTRE"       # hold over it while the estimate settles
    PAD_DESCEND = "PAD_DESCEND"     # down, correcting from the marker

    COURSE_HOLD = "COURSE_HOLD"     # hover, wait for the estimate, carry on
    # Stages that FINISH before hovering when the rangefinder drops out. They
    # are level flights across an obstacle, so the flow hold is enough to fly
    # them, and stopping in the middle would park the aircraft over the red bar
    # -- with the lidar staring at the very thing that broke the fusion, which
    # is the worst place to wait for it to come back. The vertical stages do
    # NOT get this: climbing or descending on an unanchored height estimate is
    # the thing to stop doing immediately.
    FINISH_FIRST_STAGES = ("RED_CROSS", "BLUE_CROSS", "TUBE_PASS", "TUBE_SHIFT",
                           "TUBE_EXIT", "WINDOW2_SKIP_CROSS")

    TUBE_STAGES = (TUBE_CLIMB, TUBE_APPROACH, TUBE_SCAN, TUBE_SEARCH,
                   TUBE_LOCK, TUBE_ALIGN, TUBE_PASS, TUBE_SHIFT, TUBE_EXIT)
    PAD_STAGES = (PAD_OFFSET, PAD_SEARCH, PAD_CENTRE, PAD_DESCEND)
    # The run-up to the second window. Not TUBE_STAGES (the tube detector is
    # off and there is no tube solution to update) and not the window
    # mission's own stages either -- they begin at SCAN, after this.
    WINDOW2_STAGES = (WINDOW2_RISE, WINDOW2_HOLD, WINDOW2_SKIP_RISE,
                      WINDOW2_SKIP_CROSS, WINDOW2_SKIP_DROP)
    COURSE_STAGES = ((START_OFFSET, RED_RISE, RED_CROSS, BLUE_DROP, BLUE_CROSS,
                      COURSE_HOLD) + TUBE_STAGES + WINDOW2_STAGES + PAD_STAGES)
    # Where horizontal hold is judged on FLOW FUSION ALONE. Crossing a bar or
    # the cross tube steps dist_bottom by a metre or two, EKF2 drops
    # cs_rng_kin_consistent, and it only re-earns that at |vz| > 0.5 m/s -- so
    # it never comes back in a hover. Requiring it here is what landed the
    # aircraft just after the red bar.
    FLOW_ONLY_STAGES = ((START_OFFSET, RED_CROSS, BLUE_DROP, BLUE_CROSS,
                         COURSE_HOLD)
                        + TUBE_STAGES + WINDOW2_STAGES + PAD_STAGES)

    # ---- the course layout ------------------------------------------------
    WINDOW_TO_RED = 1.00        # m, window plane to red bar
    RED_TO_BLUE = 1.00          # m, red bar to blue bar
    BLUE_EXIT = 0.80            # m past the blue bar to stop and land. The
                                # airframe is 0.26 m across, so this puts its
                                # trailing edge well clear before the descent.

    BLUE_BAR_GAP = 1.00         # m between the TWO blue bars. They are flown
                                # as one obstacle: down to the crossing height
                                # once, then straight on past both, because the
                                # second is invisible from under the first.
    TUBE_PLANE_FROM_BLUE = 1.50 # m from the SECOND BLUE BAR to the plane the
                                # tube uprights stand on. A course measurement,
                                # not a flight one.
    TUBE_LOOK_STANDOFF = 1.20   # m SHORT of that plane to stop, climb, scan
                                # and align from. Never less: this is the
                                # number that keeps the aircraft outside the
                                # obstacle while it is looking at it.
    BLUE_TO_TUBE = 0.0          # deprecated. > 0 restores the old behaviour --
                                # fly this far on from wherever BLUE_CROSS
                                # ended -- which is what drove the aircraft
                                # through the obstacle: the leg was measured
                                # from the end of the blue crossing, not from
                                # the blue bar, so blue_exit was counted twice
                                # and no standoff was taken off.

    RED_BAR_HEIGHT = 1.98
    BLUE_BAR_HEIGHT = 0.80
    BAR_RADIUS = 0.02
    RED_CLEARANCE = 0.25        # m between the landing gear and the red bar
    BLUE_CLEARANCE = 0.20       # m between the top of the aircraft and the
                                # blue bar. Tighter than red on purpose: every
                                # centimetre here comes out of the height above
                                # the optical-flow floor, and losing flow under
                                # a bar is worse than 5 cm less air above you.

    # ---- pace: safe, but not a battery drain ------------------------------
    COURSE_HOLD_SECONDS = 0.5   # s on P0 after the window before the red rise
    COURSE_SETTLE_SECONDS = 0.5 # s an altitude + position must hold before a
                                # crossing starts. Short: the gates below are
                                # tight, so a vehicle that passes them for half
                                # a second is genuinely there.
    COURSE_CLIMB_SPEED = 0.40   # m/s for the red rise
    COURSE_DESCENT_SPEED = 0.30 # m/s for the blue drop. The inherited descent
                                # rate is LAND_SPEED (0.15), sized for a
                                # touchdown; using it for a 1.9 m repositioning
                                # descent would cost ~13 s of hover. Restored
                                # before the real landing.
    BAR_CROSS_SPEED = 0.35      # m/s across each bar

    COURSE_XY_TOLERANCE = 0.12  # m, along and across, before a vertical move
                                # counts as settled on its midpoint
    COURSE_ALT_TOLERANCE = 0.08
    COURSE_VERTICAL_TIMEOUT = 25.0
    COURSE_CROSS_TIMEOUT = 15.0
    COURSE_FLOW_TIMEOUT = 8.0
    # How long the aircraft will HOVER waiting for the rangefinder or the flow
    # to come back before it gives up and lands. The lidar sweeping over the
    # red bar drops EKF2's rangefinder fusion, and the base class's answer to
    # that is to land immediately -- which is how the course ended on the
    # ground just past the red bar. Waiting is better: the aircraft is in open
    # air between obstacles, and the estimate usually comes back.
    COURSE_HOLD_TIMEOUT = 45.0
    # The rangefinder has to be healthy CONTINUOUSLY for this long before the
    # hold is over. EKF2 does not come back cleanly: cs_rng_hgt flickers true
    # for a frame or two while the fusion re-establishes itself, and resuming
    # a climb or a descent on the first true is resuming on a height estimate
    # that is about to drop out again.
    COURSE_HOLD_CONFIRM_SECONDS = 2.0
    # What happens when COURSE_HOLD_TIMEOUT runs out with the rangefinder
    # still gone. true = carry on anyway PROVIDED the EKF still has a height
    # and the flow still holds position -- which, with baro fusion enabled, is
    # the normal state of affairs during a rangefinder outage: the height is
    # the barometer's, it is good to a few centimetres over the seconds this
    # takes, and the obstacle ahead is flown on known geometry anyway.
    # false = the old behaviour, land where we are.
    COURSE_HOLD_PRESS_ON = True

    FLOW_FLOOR_MARGIN = 0.10    # m the blue altitude must clear FLOW_MIN_AGL by

    # ---- the tube obstacle (same numbers and solve as tube_cross) ----------
    TUBES = True                # false = land after the blue bars
    TUBE_SPACING = 0.50
    TUBE_RADIUS = 0.025
    CROSS_BAR_HEIGHT = 0.461
    DIAGONAL_LEFT_HEIGHT = 2.000
    DIAGONAL_RIGHT_HEIGHT = 0.922
    GAP_SIDE = 'auto'           # auto = the cell with the bigger opening,
                                # i.e. the one under the high end of the
                                # diagonal. 'left'/'right' force it.
    TUBE_LOOK_ALTITUDE = 1.10   # m to sit at while scanning. Low enough to
                                # look THROUGH the opening rather than down at
                                # it, and RAISED AUTOMATICALLY if that would
                                # put the airframe near the blue bar -- the
                                # scan happens just in front of it, and the
                                # aircraft backs up towards it to get there.
    TUBE_BLUE_CLEAR = 0.60      # m to stay in front of the second blue bar
                                # while at the scan altitude. Caps how far
                                # back the look-from point may be.
    TUBE_SCAN_YAW_DEG = 25.0    # sweep either side of the course heading
    TUBE_SCAN_RATE_DEG = 12.0   # deg/s
    TUBE_SCAN_SECONDS = 8.0     # of sweeping before choosing
    TUBE_SCAN_MAX_SECONDS = 20.0 # of sweeping before giving up on seeing
                                # anything at all and going with whatever
                                # there is
    TUBE_SCAN_SETTLE = 1.5      # s back on the course heading before choosing
    TUBE_GAP_PREFER = 'left'    # which cell wins a near-tie on area
    TUBE_GAP_TIE = 0.25         # a cell within this fraction of the biggest
                                # counts as a tie
    TUBE_EXIT_FORWARD = 1.50    # m past the tube plane where the tubes are
                                # done. The sideways step is what clears the
                                # back upright; this is how far on to go.
    TUBE_STANDOFF = 1.20        # m before the tube plane the pass starts
    TUBE_PASS_EXIT = 0.50       # m past it before stepping sideways
    TUBE_SHIFT_LEFT = 0.40      # m, + = left. The SMALLEST step sideways
                                # after the gap; where the back upright was
                                # measured, whatever it takes to clear it.
    TUBE_BACK_DISTANCE = 1.00   # m the lone back upright stands behind the plane
    TUBE_BACK_CLEAR = 1.00      # m to be past it before the course is over
    TUBE_MAX_SHIFT = 1.20       # m sideways after the gap before going the
                                # other way round the back upright instead
    TUBE_EXIT_DISTANCE = 0.0    # m from the shift point; 0 = worked out from
                                # tube_back_distance + tube_back_clear
    TUBE_LATERAL_MARGIN = 0.02  # m kept off an upright when the camera's hole
                                # centre is followed sideways
    TUBE_CROSS_DROP = 0.15      # m BELOW the centre of the opening to aim.
                                # The roof of the cell is the sloping diagonal
                                # and the floor is one horizontal tube, so
                                # dropping buys headroom against the thing
                                # that is in the way. Clamped off the floor.
    TUBE_CROSS_LEFT = 0.05      # m LEFT of the centre of the opening to aim,
                                # towards the high end of the diagonal and
                                # towards the side the aircraft leaves on.
                                # Clamped off the uprights.
    TUBE_MERGE_SHIFT = True     # fly the gap and the step round the back
                                # upright as ONE diagonal leg
    TUBE_CLEARANCE = 0.12
    TUBE_CROSS_TOLERANCE = 0.05     # m off the gap centreline: under 10 cm a side
    TUBE_ALONG_TOLERANCE = 0.15
    TUBE_YAW_TOLERANCE = math.radians(5.0)
    TUBE_ALT_TOLERANCE = 0.06
    TUBE_SETTLE_SECONDS = 1.5
    TUBE_ARRIVE_TOLERANCE = 0.10
    TUBE_CROSS_CLEAR = 0.30     # m off the exit line that still counts as
                                # having gone round the back upright
    TUBE_SEARCH_TIMEOUT = 45.0
    TUBE_LOCK_SECONDS = 2.0
    TUBE_STAGE_TIMEOUT = 40.0
    TUBE_PASS_SPEED = 0.30
    TUBE_SHIFT_SPEED = 0.25
    # ---- a second window, after the tubes ---------------------------------
    WINDOW_AFTER_TUBES = False  # true = fly the WHOLE window mission again
                                # once the tubes are done, and only then do
                                # the landing that would have followed them.
                                # The second pass is the same code as the
                                # first -- SCAN, LOCK, (RECENTRE), AIM, ALIGN,
                                # TRAVERSE, CLEAR -- restarted from wherever
                                # the tubes left the aircraft, with the window
                                # detector switched back on and the estimator
                                # emptied of everything it measured of the
                                # FIRST window. Nothing about the course
                                # geometry applies to it: it is flown entirely
                                # on what the camera measures, exactly as the
                                # first one is.
    # ---- what happens when a detector never finds its obstacle ------------
    TUBE_BACK_OFF_MAX = 0.40    # m. The look-from point is often BEHIND where
                                # the blue crossing ends, and backing all the
                                # way to it puts the aircraft over the blue
                                # bar it has just flown under -- a two-metre
                                # step in the lidar, which is the very thing
                                # that drops EKF2's rangefinder fusion. Cap
                                # the backwards move at this and make up the
                                # difference with the yaw sweep and the scan
                                # altitude retries below. 0 = uncapped.
    TUBE_SCAN_ALT_STEP = 0.25   # m the scan altitude is moved by when a whole
                                # sweep sees no opening at all. From closer in
                                # the obstacle does not fit the frame at one
                                # height; a different height often frames it.
    TUBE_SCAN_ALT_STEPS = 2     # how many such retries (up first, then down)
    TUBE_BLIND = True           # true = when nothing ever solves, cross on the
                                # KNOWN geometry instead of landing in front
                                # of the gate. See _begin_tube_blind().
    WINDOW2_SEARCH_TIMEOUT = 45.0   # s of searching for the second window
                                    # before it is skipped rather than flown
    WINDOW2_SKIP = True         # true = skip the second window (over, across,
                                # back down) if it is never found; false =
                                # land, as everything else used to do.
    WINDOW2_ALTITUDE = 0.0      # m the aircraft climbs back to before it
                                # starts looking for the second window;
                                # 0 = takeoff_altitude, the height the first
                                # window was searched for from. The tubes end
                                # at the gap altitude, which is BELOW the
                                # cross tube's height and below anything a
                                # window is hung at, and searching from down
                                # there is what flew it into the obstacle.
    WINDOW2_HOLD_SECONDS = 2.0  # s stationary at that altitude before the
                                # search starts, so the climb has stopped
                                # moving the camera before a single frame is
                                # measured.
    WINDOW2_EXIT_DISTANCE = 0.0 # m beyond the second window the traverse
                                # ends; 0 = keep exit_distance, which the
                                # course has already pinned to the midpoint of
                                # the window-to-red gap. There is no red bar
                                # after this window, so it can be longer.
    # ---- the landing pad, after the tubes ---------------------------------
    PAD = True                  # false = land where the tubes finish
    # The takeoff pad does not face the window: from where the aircraft is
    # put down, the window is off to one side and the camera cannot see it at
    # all. So the flight steps sideways first and only then starts looking.
    # Positive is to the RIGHT of the arming heading; negative steps left.
    START_OFFSET_RIGHT = 1.00   # m sideways off the pad before the sweep
    START_OFFSET_TIMEOUT = 30.0 # s before the step is called done regardless

    PAD_RIGHT = 1.50            # m to the RIGHT after the tubes, off the line
                                # the obstacles stand on
    PAD_SEARCH_DISTANCE = 6.00  # m of forward creep before giving up
    PAD_SEARCH_SPEED = 0.30
    PAD_CENTRE_TOLERANCE = 0.10 # m from the marker before the descent starts
    PAD_CENTRE_SECONDS = 1.0
    PAD_DESCENT_RATE = 0.20     # m/s the setpoint walks down at
    PAD_HANDOFF_HEIGHT = 0.45   # m above the pad where PX4's land takes over:
                                # below this the marker fills the frame and
                                # then leaves it altogether
    PAD_GAIN = 0.8              # of the measured offset, per correction
    PAD_MAX_NUDGE = 0.30        # m the target may be moved in one correction
    PAD_LOST_SECONDS = 2.0      # of no marker before the descent stops
    PAD_STAGE_TIMEOUT = 60.0
    TIMER_PERIOD = 0.05         # s, the base class's loop. The descent walks
                                # the setpoint down by rate * this each tick.
    ARUCO_DETECT_TOPIC = '/aruco/detected'
    ARUCO_POINT_TOPIC = '/aruco/point'

    TUBE_GEOMETRY_TOPIC = 'tube_geometry'
    TUBE_DETECT_TOPIC = 'tubes_detected'
    TUBE_HOLE_TOPIC = 'tube_hole'
    TUBE_DEPTH_MIN = 0.40
    TUBE_DEPTH_MAX = 6.00
    TUBE_MIN_TOP_HEIGHT = 1.20
    TUBE_MAX_BOTTOM_HEIGHT = 0.40
    TUBE_BUFFER_SECONDS = 2.5
    TUBE_BUFFER_MAX = 400
    TUBE_CLUSTER_RADIUS = 0.15
    TUBE_MIN_SAMPLES = 6
    TUBE_PLANE_BAND = 0.40
    TUBE_MATCH_TOLERANCE = 0.12
    TUBE_MIN_MATCHED = 3
    TUBE_MAX_PLANE_YAW_DEG = 30.0
    TUBE_MIN_HOLE_WIDTH = 0.30  # m. Under this it is not a whole cell --
                                # something was standing in front of it.
    TUBE_MIN_HOLE_HEIGHT = 0.50 # m, measured over the aircraft's own track
    TUBE_MIN_GAP_WIDTH = 0.40
    TUBE_MAX_GAP_WIDTH = 0.60

    # The window mission ends ON the first midpoint, and the whole course fits
    # inside this clock (crossings excluded, as for the window traverse).
    EXIT_DISTANCE = 0.50
    FLIGHT_SECONDS = 240.0

    def __init__(self):
        super().__init__(node_name='course_fsm')

        n = self._declare_number
        self.WINDOW_TO_RED = float(n('window_to_red_distance', self.WINDOW_TO_RED))
        self.RED_TO_BLUE = float(n('red_to_blue_distance', self.RED_TO_BLUE))
        self.BLUE_EXIT = float(n('blue_exit_distance', self.BLUE_EXIT))
        self.RED_BAR_HEIGHT = float(n('red_bar_height', self.RED_BAR_HEIGHT))
        self.BLUE_BAR_HEIGHT = float(n('blue_bar_height', self.BLUE_BAR_HEIGHT))
        self.BAR_RADIUS = float(n('bar_radius', self.BAR_RADIUS))
        self.RED_CLEARANCE = float(n('red_clearance', self.RED_CLEARANCE))
        self.BLUE_CLEARANCE = float(n('blue_clearance', self.BLUE_CLEARANCE))
        self.COURSE_HOLD_SECONDS = float(n('course_hold_seconds', self.COURSE_HOLD_SECONDS))
        self.COURSE_SETTLE_SECONDS = float(n(
            'course_settle_seconds', self.COURSE_SETTLE_SECONDS))
        self.COURSE_CLIMB_SPEED = float(n('course_climb_speed', self.COURSE_CLIMB_SPEED))
        self.COURSE_DESCENT_SPEED = float(n(
            'course_descent_speed', self.COURSE_DESCENT_SPEED))
        self.BAR_CROSS_SPEED = float(n('bar_cross_speed', self.BAR_CROSS_SPEED))
        self.COURSE_XY_TOLERANCE = float(n('course_xy_tolerance', self.COURSE_XY_TOLERANCE))
        self.COURSE_ALT_TOLERANCE = float(n(
            'course_alt_tolerance', self.COURSE_ALT_TOLERANCE))
        self.COURSE_VERTICAL_TIMEOUT = float(n(
            'course_vertical_timeout', self.COURSE_VERTICAL_TIMEOUT))
        self.COURSE_CROSS_TIMEOUT = float(n(
            'course_cross_timeout', self.COURSE_CROSS_TIMEOUT))
        self.COURSE_FLOW_TIMEOUT = float(n('course_flow_timeout', self.COURSE_FLOW_TIMEOUT))
        self.COURSE_HOLD_TIMEOUT = float(n('course_hold_timeout', self.COURSE_HOLD_TIMEOUT))
        self.COURSE_HOLD_CONFIRM_SECONDS = float(n(
            'course_hold_confirm_seconds', self.COURSE_HOLD_CONFIRM_SECONDS))
        self.COURSE_HOLD_PRESS_ON = bool(self.declare_parameter(
            'course_hold_press_on', self.COURSE_HOLD_PRESS_ON).value)
        self.course_resume_stage = None
        self.course_hold_since = None
        self.course_hold_reason = ''

        self.red_altitude = (self.RED_BAR_HEIGHT + self.BAR_RADIUS
                             + self.RED_CLEARANCE + self.body_below)
        self.blue_altitude = (self.BLUE_BAR_HEIGHT - self.BAR_RADIUS
                              - self.BLUE_CLEARANCE - self.body_above)

        # The traverse must end on the first midpoint. Anything further is
        # closer to the red bar than the course allows a vertical move to be.
        midpoint = 0.5 * self.WINDOW_TO_RED
        if abs(self.EXIT_DISTANCE - midpoint) > 1e-3:
            self.get_logger().warning(
                f"exit_distance was {self.EXIT_DISTANCE:.2f} m; the course needs "
                f"the traverse to end on the midpoint of the "
                f"{self.WINDOW_TO_RED:.2f} m window-to-red gap, so it is "
                f"{midpoint:.2f} m.")
            self.EXIT_DISTANCE = midpoint
        self._blind_seconds_param = self.BLIND_TRAVERSE_SECONDS

        self.course_problems = self._check_course()
        for problem in self.course_problems:
            self.get_logger().error(f"COURSE NOT FLYABLE: {problem}")

        n = self._declare_number
        self.BLUE_BAR_GAP = float(n('blue_bar_gap', self.BLUE_BAR_GAP))
        self.BLUE_TO_TUBE = float(n('blue_to_tube_distance', self.BLUE_TO_TUBE))
        self.TUBE_PLANE_FROM_BLUE = float(n('tube_plane_from_blue',
                                            self.TUBE_PLANE_FROM_BLUE))
        self.TUBE_LOOK_STANDOFF = float(n('tube_look_standoff',
                                          self.TUBE_LOOK_STANDOFF))
        self.TUBES = bool(self.declare_parameter('tubes', self.TUBES).value)
        self.TUBE_SPACING = float(n('tube_spacing', self.TUBE_SPACING))
        self.TUBE_RADIUS = float(n('tube_radius', self.TUBE_RADIUS))
        self.CROSS_BAR_HEIGHT = float(n('cross_bar_height', self.CROSS_BAR_HEIGHT))
        self.DIAGONAL_LEFT_HEIGHT = float(n('diagonal_left_height', self.DIAGONAL_LEFT_HEIGHT))
        self.DIAGONAL_RIGHT_HEIGHT = float(n('diagonal_right_height', self.DIAGONAL_RIGHT_HEIGHT))
        side = str(self.declare_parameter('gap_side', self.GAP_SIDE).value).strip().lower()
        self.gap_side = side if side in ('left', 'right', 'auto') else self.GAP_SIDE
        self.TUBE_STANDOFF = float(n('tube_standoff', self.TUBE_STANDOFF))
        self.TUBE_PASS_EXIT = float(n('tube_pass_exit', self.TUBE_PASS_EXIT))
        self.TUBE_SHIFT_LEFT = float(n('tube_shift_left', self.TUBE_SHIFT_LEFT))
        self.TUBE_BACK_DISTANCE = float(n('tube_back_distance', self.TUBE_BACK_DISTANCE))
        self.TUBE_BACK_CLEAR = float(n('tube_back_clear', self.TUBE_BACK_CLEAR))
        self.TUBE_MAX_SHIFT = float(n('tube_max_shift', self.TUBE_MAX_SHIFT))
        self.TUBE_LOOK_ALTITUDE = float(n('tube_look_altitude', self.TUBE_LOOK_ALTITUDE))
        self.TUBE_BLUE_CLEAR = float(n('tube_blue_clear', self.TUBE_BLUE_CLEAR))
        self.TUBE_RAISE_OVER_BLUE = bool(self.declare_parameter(
            'tube_raise_over_blue', False).value)
        self.TUBE_SCAN_YAW = math.radians(float(n(
            'tube_scan_yaw_deg', self.TUBE_SCAN_YAW_DEG)))
        self.TUBE_SCAN_RATE = math.radians(float(n(
            'tube_scan_rate_deg', self.TUBE_SCAN_RATE_DEG)))
        self.TUBE_SCAN_SECONDS = float(n('tube_scan_seconds', self.TUBE_SCAN_SECONDS))
        self.TUBE_SCAN_MAX_SECONDS = float(n('tube_scan_max_seconds',
                                             self.TUBE_SCAN_MAX_SECONDS))
        self.TUBE_EXIT_FORWARD = float(n('tube_exit_forward', self.TUBE_EXIT_FORWARD))
        prefer = str(self.declare_parameter(
            'tube_gap_prefer', self.TUBE_GAP_PREFER).value).strip().lower()
        self.TUBE_GAP_PREFER = prefer if prefer in ('left', 'right', 'none') else 'left'
        self.TUBE_GAP_TIE = float(n('tube_gap_tie', self.TUBE_GAP_TIE))
        self.TUBE_ALLOW_SHIFT_RIGHT = bool(self.declare_parameter(
            'tube_allow_shift_right', False).value)

        self.WINDOW_AFTER_TUBES = bool(self.declare_parameter(
            'window_after_tubes', self.WINDOW_AFTER_TUBES).value)
        self.WINDOW2_EXIT_DISTANCE = float(n('window2_exit_distance',
                                             self.WINDOW2_EXIT_DISTANCE))
        self.TUBE_BACK_OFF_MAX = float(n('tube_back_off_max', self.TUBE_BACK_OFF_MAX))
        self.TUBE_SCAN_ALT_STEP = float(n('tube_scan_alt_step', self.TUBE_SCAN_ALT_STEP))
        self.TUBE_SCAN_ALT_STEPS = int(n('tube_scan_alt_steps', self.TUBE_SCAN_ALT_STEPS))
        self.TUBE_BLIND = bool(self.declare_parameter(
            'tube_blind', self.TUBE_BLIND).value)
        self.WINDOW2_SEARCH_TIMEOUT = float(n('window2_search_timeout',
                                              self.WINDOW2_SEARCH_TIMEOUT))
        self.WINDOW2_SKIP = bool(self.declare_parameter(
            'window2_skip', self.WINDOW2_SKIP).value)
        self.WINDOW2_ALTITUDE = float(n('window2_altitude',
                                        self.WINDOW2_ALTITUDE))
        if self.WINDOW2_ALTITUDE <= 0.0:
            self.WINDOW2_ALTITUDE = self.TAKEOFF_ALTITUDE
        self.WINDOW2_HOLD_SECONDS = float(n('window2_hold_seconds',
                                            self.WINDOW2_HOLD_SECONDS))

        self.START_OFFSET_RIGHT = float(n('start_offset_right',
                                          self.START_OFFSET_RIGHT))
        self.START_OFFSET_TIMEOUT = float(n('start_offset_timeout',
                                            self.START_OFFSET_TIMEOUT))

        self.PAD = bool(self.declare_parameter('pad', self.PAD).value)
        self.PAD_RIGHT = float(n('pad_right', self.PAD_RIGHT))
        self.PAD_SEARCH_DISTANCE = float(n('pad_search_distance', self.PAD_SEARCH_DISTANCE))
        self.PAD_SEARCH_SPEED = float(n('pad_search_speed', self.PAD_SEARCH_SPEED))
        self.PAD_CENTRE_TOLERANCE = float(n('pad_centre_tolerance', self.PAD_CENTRE_TOLERANCE))
        self.PAD_DESCENT_RATE = float(n('pad_descent_rate', self.PAD_DESCENT_RATE))
        self.PAD_HANDOFF_HEIGHT = float(n('pad_handoff_height', self.PAD_HANDOFF_HEIGHT))
        self.PAD_GAIN = float(n('pad_gain', self.PAD_GAIN))
        self.PAD_MAX_NUDGE = float(n('pad_max_nudge', self.PAD_MAX_NUDGE))
        self.PAD_LOST_SECONDS = float(n('pad_lost_seconds', self.PAD_LOST_SECONDS))
        self.TUBE_EXIT_DISTANCE = float(n('tube_exit_distance', self.TUBE_EXIT_DISTANCE))
        self.TUBE_LATERAL_MARGIN = float(n('tube_lateral_margin', self.TUBE_LATERAL_MARGIN))
        self.TUBE_CROSS_DROP = float(n('tube_cross_drop', self.TUBE_CROSS_DROP))
        self.TUBE_CROSS_LEFT = float(n('tube_cross_left', self.TUBE_CROSS_LEFT))
        self.TUBE_MERGE_SHIFT = bool(self.declare_parameter(
            'tube_merge_shift', self.TUBE_MERGE_SHIFT).value)
        self.TUBE_CLEARANCE = float(n('tube_clearance', self.TUBE_CLEARANCE))
        self.TUBE_CROSS_TOLERANCE = float(n('tube_cross_tolerance', self.TUBE_CROSS_TOLERANCE))
        self.TUBE_SEARCH_TIMEOUT = float(n('tube_search_timeout', self.TUBE_SEARCH_TIMEOUT))
        self.TUBE_MAX_PLANE_YAW = math.radians(float(n(
            'tube_max_plane_yaw_deg', self.TUBE_MAX_PLANE_YAW_DEG)))
        self.TUBE_MIN_MATCHED = int(n('tube_min_matched', self.TUBE_MIN_MATCHED))

        self.tube_estimator = TubeEstimator(
            depth_min=float(n('tube_depth_min', self.TUBE_DEPTH_MIN)),
            depth_max=float(n('tube_depth_max', self.TUBE_DEPTH_MAX)),
            tube_radius=self.TUBE_RADIUS,
            min_top_height=float(n('min_tube_top_height', self.TUBE_MIN_TOP_HEIGHT)),
            max_bottom_height=float(n('max_tube_bottom_height', self.TUBE_MAX_BOTTOM_HEIGHT)),
            buffer_seconds=float(n('tube_buffer_seconds', self.TUBE_BUFFER_SECONDS)),
            buffer_max=self.TUBE_BUFFER_MAX,
            cluster_radius=float(n('tube_cluster_radius', self.TUBE_CLUSTER_RADIUS)),
            min_samples=int(n('tube_min_samples', self.TUBE_MIN_SAMPLES)),
            min_hole_width=float(n('tube_min_hole_width', self.TUBE_MIN_HOLE_WIDTH)),
            min_hole_height=float(n('tube_min_hole_height', self.TUBE_MIN_HOLE_HEIGHT)),
            max_hole_floor=float(n('tube_max_hole_floor',
                                   self._tube_default_hole_floor())),
        )
        self.tube_altitude, self.tube_floor, self.tube_roof = self._tube_solve_altitude()
        self.TUBE_LOOK_ALTITUDE = self._tube_look_altitude()

        self.create_subscription(
            Float32MultiArray,
            str(self.declare_parameter('tube_geometry_topic', self.TUBE_GEOMETRY_TOPIC).value),
            self.tube_geometry_callback, 10, callback_group=self.sensor_cbg)
        self.create_subscription(
            Bool, str(self.declare_parameter('tube_detect_topic', self.TUBE_DETECT_TOPIC).value),
            self.tube_detected_callback, 10, callback_group=self.sensor_cbg)
        self.create_subscription(
            Float32MultiArray,
            str(self.declare_parameter('tube_hole_topic', self.TUBE_HOLE_TOPIC).value),
            self.tube_hole_callback, 10, callback_group=self.sensor_cbg)
        self.tube_gap_pub = self.create_publisher(String, 'tube_gap', 10)

        # Detection runs only for the phase it belongs to. Two reasons: the
        # CPU, and the exit wall's RED window, which is a red rectangle of
        # about the right size in front of a tube detector that is looking for
        # red. Latched so a detector that starts late still gets the state.
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.detection_pubs = {
            'window': self.create_publisher(
                Bool, str(self.declare_parameter(
                    'window_enable_topic', '/window_detect/enable').value), latched),
            'tube': self.create_publisher(
                Bool, str(self.declare_parameter(
                    'tube_enable_topic', '/tube_detect/enable').value), latched),
        }
        self.detection_on = {}
        # Window detection is wanted from the start; tube detection is not,
        # and leaving it on through the window and the bars is what lets it
        # find "a tube obstacle" in the exit wall's red window.
        self.set_detection('window', True)
        self.set_detection('tube', False)

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
        self.aruco_seen = 0
        self.tubes_done = False
        self.scan_since = 0.0
        self.scan_ended = None
        self.scan_choice = None
        self.scan_report = None
        self.start_offset_done = False
        self.pad_search_start = None
        self.pad_settle_since = None
        self.pad_lost_since = None

        self.tubes_flag = False
        self.tube_geometry_seen = 0
        self.tube_solution = None
        self.tube_reason = 'no data yet'
        self.tube_ok_since = None
        self.gap_point = self.gap_normal = self.gap_left = None
        self.gap_heading = None
        self.tube_entry = self.tube_pass_exit = None
        self.tube_back_along = self.TUBE_BACK_DISTANCE
        self.tube_back_lateral = 0.0
        self.tube_shift = self.TUBE_SHIFT_LEFT
        self.tube_shift_point = self.tube_final_point = None
        self.tube_settle_since = None
        self.tube_push_since = None
        self.tube_push_seconds = 0.0

        # 1 while the window before the bars is being flown, 2 once the
        # tubes are done and the second pass has been started.
        self.window_pass = 1
        self.window2_skipped = False
        self.tube_scan_alt_tries = 0
        self.tube_blind_flown = False
        self.tube_plane_ahead = None    # m from the look-from point to the
                                        # gate plane, on the course reckoning

        self.course_saved_land_speed = None
        self.course_rng_ok_since = None
        self.course_pressed_on = False  # a hold has already been resolved by
                                        # pressing on without the rangefinder
        self.course_settle_since = None
        self.course_flow_lost_since = None
        self.bar_push_since = None
        self.bar_push_seconds = 0.0
        self.window_blind_capped = False

        self.get_logger().warning(
            f"COURSE: window, then OVER the red bar ({self.RED_BAR_HEIGHT:.2f} m, "
            f"cross at {self.red_altitude:.2f} m), then UNDER the blue bar "
            f"({self.BLUE_BAR_HEIGHT:.2f} m, cross at {self.blue_altitude:.2f} m), "
            f"then land {self.BLUE_EXIT:.2f} m past it. Gaps "
            f"{self.WINDOW_TO_RED:.2f} / {self.RED_TO_BLUE:.2f} m; every "
            "vertical move happens at a gap midpoint. Bars are flown on the "
            "known geometry along the window's traverse line. "
            + (f"Then {self.BLUE_TO_TUBE:.2f} m on to the tubes, gap at "
               f"{self.tube_altitude:.2f} m (band {self.tube_floor:.2f}-"
               f"{self.tube_roof:.2f} m), aiming {self.TUBE_CROSS_DROP:.2f} m "
               f"under and {self.TUBE_CROSS_LEFT:.2f} m left of the middle of "
               f"the opening. " if self.TUBES else "Tubes disabled. ")
            + (f"Then back up to {self.WINDOW2_ALTITUDE:.2f} m, a "
               f"{self.WINDOW2_HOLD_SECONDS:.1f} s hover, the whole window "
               "mission again on the next window, and the landing after "
               "that. " if self.WINDOW_AFTER_TUBES else "")
            + ("READY." if not self.course_problems else
               "The bars will NOT be attempted -- the aircraft lands after the "
               "window. See the errors above."))

    def _check_course(self):
        """Everything that would make the bars unflyable, found on the ground."""
        problems = []
        if self.red_altitude > self.MAX_ALTITUDE:
            problems.append(
                f"the red crossing needs {self.red_altitude:.2f} m but "
                f"max_altitude is {self.MAX_ALTITUDE:.2f} m")
        if self.blue_altitude < self.MIN_ALTITUDE:
            problems.append(
                f"the blue crossing needs {self.blue_altitude:.2f} m but "
                f"min_altitude is {self.MIN_ALTITUDE:.2f} m")
        if self.blue_altitude < self.FLOW_MIN_AGL + self.FLOW_FLOOR_MARGIN:
            problems.append(
                f"the blue crossing at {self.blue_altitude:.2f} m is within "
                f"{self.FLOW_FLOOR_MARGIN:.2f} m of the optical-flow floor "
                f"({self.FLOW_MIN_AGL:.2f} m) -- flow would drop out under the bar")
        half = 0.5 * min(self.WINDOW_TO_RED, self.RED_TO_BLUE)
        if half - 0.5 * self.DRONE_WIDTH < 0.25:
            problems.append(
                f"a {2 * half:.2f} m gap leaves only "
                f"{half - 0.5 * self.DRONE_WIDTH:.2f} m from a prop tip to an "
                "obstacle at the midpoint")
        return problems

    # ---------------------------------------------------------- geometry

    def _course_heading(self):
        """The direction the course runs in.

        traverse_heading is the window's, and it does not exist until the
        window has been measured -- which is only after START_OFFSET, the
        one course stage that runs BEFORE the window. Every consumer of this
        (the setpoint geometry, _target_errors, publish_status) takes the
        answer straight into math.cos, so returning None there is a crash
        rather than a missing number. Before the window, the course line is
        the heading the aircraft was armed on, which is what START_OFFSET is
        flown relative to anyway.
        """
        return self.home_yaw if self.traverse_heading is None else self.traverse_heading

    def _target_errors(self):
        """(along, cross) from the aircraft to the current move target.

        along is positive while the target is still AHEAD on the course line.
        Measured against move_target, which the base class already shifts on
        EKF2 lateral resets and WindowTraverse rotates on heading resets, so
        the course needs no reset handling of its own.
        """
        lp = self.local_position
        if lp is None or self.move_target_x is None:
            return None, None
        h = self._course_heading()
        ex = self.move_target_x - lp.x
        ey = self.move_target_y - lp.y
        c, s = math.cos(h), math.sin(h)
        return ex * c + ey * s, -ex * s + ey * c

    def _ahead(self, distance):
        """The move target pushed `distance` further down the course line."""
        h = self._course_heading()
        return (self.move_target_x + distance * math.cos(h),
                self.move_target_y + distance * math.sin(h))

    # --------------------------------------------------- window -> course

    def _handle_blind_traverse(self):
        """The window's blind push, capped so it cannot reach the red bar.

        Inherited, it pushes for blind_traverse_seconds regardless of where it
        started -- up to 1.35 m, which on this course is into the red bar. At
        the moment flow is lost, cap the push to the distance left to the end
        of the traverse (P0), then let the inherited logic run it and land.
        """
        if self.blind_traverse_since is None:
            along = self._distance_along_traverse()
            total = self.STANDOFF_DISTANCE + self.EXIT_DISTANCE
            remaining = max(0.0, total - along)
            cap = remaining / max(self.TRAVERSE_SPEED, 1e-3)
            self.BLIND_TRAVERSE_SECONDS = min(self._blind_seconds_param, cap)
            self.get_logger().error(
                f"COURSE: blind push capped to {self.BLIND_TRAVERSE_SECONDS:.1f} s "
                f"({remaining:.2f} m to the midpoint before the red bar) so it "
                "cannot carry the aircraft into the red bar.")
        super()._handle_blind_traverse()

    def _handle_clear(self):
        """Through the window: hand off to the bars instead of landing.

        On the SECOND pass (window_after_tubes) there is nothing left to hand
        off to -- the bars and the tubes are behind us -- so it finishes the
        flight the way the tubes would have: the pad, or a landing.
        """
        if self.window_pass >= 2:
            if not self._still_flyable():
                return
            self._try_latch_xy_hold()
            if self._in_stage_for() >= self.COURSE_HOLD_SECONDS:
                self.set_detection('window', False)
                self._after_the_course("second window traversed")
            return
        if self.course_problems:
            super()._handle_clear()
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        if self._in_stage_for() >= self.COURSE_HOLD_SECONDS:
            self._begin_red_rise()

    # ------------------------------------------------------------ stages

    def _enter_course_stage(self, stage):
        self._enter_stage(stage)
        self.course_settle_since = None
        self.course_flow_lost_since = None
        self.bar_push_since = None

    def _begin_red_rise(self):
        # The window is behind us. Nothing downstream reads /window_geometry
        # -- the bars are flown from their measured heights, blind -- so the
        # window detector has no work left to do and stops here.
        self.set_detection('window', False)
        # P0: the end of the traverse line. Using the frozen exit point rather
        # than wherever the aircraft stopped puts the climb on the centreline,
        # so any drift during the traverse is corrected here, on the midpoint.
        if self.traverse_exit is not None:
            x, y = float(self.traverse_exit[0]), float(self.traverse_exit[1])
        else:
            lp = self.local_position
            x, y = lp.x, lp.y
        self.CLIMB_SPEED = self.COURSE_CLIMB_SPEED
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(x, y, self.red_altitude)
        self._enter_course_stage(self.RED_RISE)
        self.get_logger().warning(
            f"RED_RISE: climbing straight up to {self.red_altitude:.2f} m on the "
            f"midpoint, {0.5 * self.WINDOW_TO_RED:.2f} m short of the red bar, "
            "holding position on the window centreline.")

    def _begin_red_cross(self):
        distance = 0.5 * self.WINDOW_TO_RED + 0.5 * self.RED_TO_BLUE
        x, y = self._ahead(distance)
        self.MOVE_SPEED = self.BAR_CROSS_SPEED
        self._set_target(x, y, self.red_altitude)
        self._enter_course_stage(self.RED_CROSS)
        self.get_logger().warning(
            f"RED_CROSS: over the red bar, {distance:.2f} m to the next midpoint "
            f"at {self.BAR_CROSS_SPEED:.2f} m/s, altitude {self.red_altitude:.2f} m.")

    def _begin_blue_drop(self):
        # The blue bar is 73 degrees below the camera from up here and nothing
        # about it can be seen. Nothing about it needs to be: descend straight
        # down on the midpoint to its crossing altitude.
        x, y = self.move_target_x, self.move_target_y
        self.course_saved_land_speed = self.LAND_SPEED
        self.LAND_SPEED = self.COURSE_DESCENT_SPEED
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(x, y, self.blue_altitude)
        self._enter_course_stage(self.BLUE_DROP)
        self.get_logger().warning(
            f"BLUE_DROP: descending straight down to {self.blue_altitude:.2f} m "
            f"on the midpoint at {self.COURSE_DESCENT_SPEED:.2f} m/s, "
            f"{0.5 * self.RED_TO_BLUE:.2f} m clear of both bars.")

    def _begin_blue_cross(self):
        self._restore_land_speed()
        # Both blue bars in one run: the second is invisible from under the
        # first, and there is no room to stop between them.
        distance = 0.5 * self.RED_TO_BLUE + self.BLUE_BAR_GAP + self.BLUE_EXIT
        x, y = self._ahead(distance)
        self.MOVE_SPEED = self.BAR_CROSS_SPEED
        self._set_target(x, y, self.blue_altitude)
        self._enter_course_stage(self.BLUE_CROSS)
        self.get_logger().warning(
            f"BLUE_CROSS: under BOTH blue bars, {distance:.2f} m at "
            f"{self.BAR_CROSS_SPEED:.2f} m/s, altitude {self.blue_altitude:.2f} m.")

    def _restore_land_speed(self):
        if self.course_saved_land_speed is not None:
            self.LAND_SPEED = self.course_saved_land_speed
            self.course_saved_land_speed = None

    def _begin_landing(self, reason):
        # Whatever ends the flight, the touchdown uses the real landing rate,
        # never the faster repositioning descent the blue drop borrowed.
        self._restore_land_speed()
        super()._begin_landing(reason)

    # ------------------------------------------------------ the handlers

    def _flow_lost_for(self):
        if self.hold_xy:
            self.course_flow_lost_since = None
            return 0.0
        now = time.monotonic()
        if self.course_flow_lost_since is None:
            self.course_flow_lost_since = now
            self.get_logger().error(
                f"{self.current_stage}: optical flow lost -- no position hold.")
        return now - self.course_flow_lost_since

    def _handle_vertical(self, next_stage_begin, what):
        """RED_RISE and BLUE_DROP: straight up or down on a midpoint."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        self._aim_yaw_at(self._course_heading())

        lost = self._flow_lost_for()
        if lost > 0.0:
            self.course_settle_since = None
            if lost > self.COURSE_FLOW_TIMEOUT:
                self._hold_and_wait(self.current_stage,
                                    f"optical flow lost during {what}")
            return

        along, cross = self._target_errors()
        alt = self.relative_altitude()
        settled = (along is not None and alt is not None
                   and abs(along) <= self.COURSE_XY_TOLERANCE
                   and abs(cross) <= self.COURSE_XY_TOLERANCE
                   and abs(alt - self.commanded_altitude) <= self.COURSE_ALT_TOLERANCE)

        if settled:
            now = time.monotonic()
            if self.course_settle_since is None:
                self.course_settle_since = now
            elif now - self.course_settle_since >= self.COURSE_SETTLE_SECONDS:
                next_stage_begin()
            return
        self.course_settle_since = None

        if self._in_stage_for() > self.COURSE_VERTICAL_TIMEOUT:
            self._abandon(
                f"{what} did not settle in {self.COURSE_VERTICAL_TIMEOUT:.0f} s")
            return

        self.get_logger().info(
            f"{self.current_stage}: alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.commanded_altitude:.2f} m, "
            f"{0.0 if along is None else along:+.2f} m along / "
            f"{0.0 if cross is None else cross:+.2f} m across the midpoint "
            f"(tolerance {self.COURSE_XY_TOLERANCE:.2f}).",
            throttle_duration_sec=1.0)

    def _handle_red_rise(self):
        self._handle_vertical(self._begin_red_cross, "the red rise")

    def _handle_blue_drop(self):
        self._handle_vertical(self._begin_blue_cross, "the blue drop")

    def _handle_red_cross(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        self._aim_yaw_at(self._course_heading())
        now = time.monotonic()

        if not self.hold_xy:
            # Above the red bar with no position. Descending here lands on the
            # bar and stopping here hovers over it, so push on, open loop, for
            # exactly the distance that was left, and land past it.
            if self.bar_push_since is None:
                along, _ = self._target_errors()
                remaining = max(0.0, along or 0.0)
                self.bar_push_seconds = remaining / max(self.BAR_CROSS_SPEED, 1e-3)
                self.bar_push_since = now
                self.get_logger().error(
                    f"RED_CROSS: flow lost over the red bar. Pushing on open-loop "
                    f"for {self.bar_push_seconds:.1f} s ({remaining:.2f} m) to get "
                    "past it rather than descending onto it.")
            elif now - self.bar_push_since >= self.bar_push_seconds:
                self._hold_and_wait(
                    self.BLUE_DROP,
                    "flow lost over the red bar; pushed clear of it open-loop")
            return
        if self.bar_push_since is not None:
            self.get_logger().warning("RED_CROSS: flow is back; resuming the crossing.")
            self.bar_push_since = None

        self._handle_crossing(self._begin_blue_drop, "the red crossing")

    def _handle_blue_cross(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        self._aim_yaw_at(self._course_heading())

        if not self.hold_xy:
            # Under the bars: push on for what is left rather than stopping in
            # the gap, then hover clear of them and wait.
            now = time.monotonic()
            if self.bar_push_since is None:
                along, _ = self._target_errors()
                remaining = max(0.0, along or 0.0)
                self.bar_push_seconds = remaining / max(self.BAR_CROSS_SPEED, 1e-3)
                self.bar_push_since = now
                self.get_logger().error(
                    f"BLUE_CROSS: flow lost under the bars. Pushing on "
                    f"{remaining:.2f} m open-loop, then hovering.")
            elif now - self.bar_push_since >= self.bar_push_seconds:
                self._hold_and_wait(self.BLUE_CROSS,
                                    "flow lost under the blue bars")
            return
        self.bar_push_since = None

        self._handle_crossing(self._finish_course, "the blue crossing")

    def _handle_crossing(self, done, what):
        along, cross = self._target_errors()
        if along is not None and along <= self.COURSE_XY_TOLERANCE:
            if self._range_ok_or_hold():
                done()
            return
        if self._in_stage_for() > self.COURSE_CROSS_TIMEOUT:
            self._abandon(f"{what} timed out {along or 0.0:.2f} m short")
            return
        self.get_logger().info(
            f"{self.current_stage}: {0.0 if along is None else along:.2f} m to go, "
            f"{0.0 if cross is None else cross:+.2f} m off the line.",
            throttle_duration_sec=0.5)

    def _finish_course(self):
        self.outcome = (f"BARS COMPLETE: window, over red at "
                        f"{self.red_altitude:.2f} m, under both blue at "
                        f"{self.blue_altitude:.2f} m")
        if not self.TUBES:
            self._begin_landing("course complete (tubes disabled)")
            return
        self._begin_tube_climb()

    # --------------------------------------------------------------- TUBES

    def _tube_default_hole_floor(self):
        """The highest a flyable cell's FLOOR can be: halfway between the
        cross tube and the LOW end of the diagonal.

        Every cell the aircraft may cross rests on the cross tube. The cells
        ABOVE the diagonal -- which exist as soon as the top of the frame is
        sealed so the tall cell can be seen at all -- rest on the diagonal,
        and the lowest the diagonal ever gets is its low end. Halfway between
        the two separates them on this obstacle and on a scaled copy of it
        alike, with no number to keep in step by hand.
        """
        low = min(self.DIAGONAL_LEFT_HEIGHT, self.DIAGONAL_RIGHT_HEIGHT)
        return 0.5 * (self.CROSS_BAR_HEIGHT + max(low, self.CROSS_BAR_HEIGHT))

    def _tube_diagonal_height(self, lateral):
        mid = 0.5 * (self.DIAGONAL_LEFT_HEIGHT + self.DIAGONAL_RIGHT_HEIGHT)
        slope = ((self.DIAGONAL_LEFT_HEIGHT - self.DIAGONAL_RIGHT_HEIGHT)
                 / (2.0 * self.TUBE_SPACING))
        return mid + slope * lateral

    def _tube_cell(self, side):
        """(area, lateral, height) of the open cell one side of the middle tube.

        Columns across the cell: floor the cross tube, roof the sloping
        diagonal, a tube radius off every edge. What comes back is its centre
        of AREA -- the middle of the trapezium. The middle of the two uprights
        at the middle of the altitude band is NOT that point: it sits low and
        over towards the low end of the diagonal, which is how the aircraft
        ends up skimming it.
        """
        sign = 1.0 if side == 'left' else -1.0
        lo, hi = sorted((self.TUBE_RADIUS * sign,
                         sign * (self.TUBE_SPACING - self.TUBE_RADIUS)))
        bottom = self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS
        steps = 60
        dx = (hi - lo) / steps
        area = lat = hgt = 0.0
        for i in range(steps):
            x = lo + (i + 0.5) * dx
            top = self._tube_diagonal_height(x) - self.TUBE_RADIUS
            column = max(0.0, top - bottom) * dx
            area += column
            lat += x * column
            hgt += 0.5 * (top + bottom) * column
        if area <= 0.0:
            return 0.0, sign * 0.5 * self.TUBE_SPACING, bottom
        return area, lat / area, hgt / area

    def _tube_solve_altitude(self):
        """Crossing height for the gap, solved across the whole airframe."""
        if self.gap_side == 'auto':
            self.gap_side = max(('left', 'right'), key=lambda c: self._tube_cell(c)[0])
            self.get_logger().info(
                f"gap_side=auto -> the {self.gap_side.upper()} cell is the "
                f"bigger opening ({self._tube_cell(self.gap_side)[0]:.2f} m2).")
        _, self.tube_cell_lateral, self.tube_cell_height = self._tube_cell(self.gap_side)
        edges = (self.tube_cell_lateral - 0.5 * self.DRONE_WIDTH,
                 self.tube_cell_lateral + 0.5 * self.DRONE_WIDTH)
        roof_tube = min(self._tube_diagonal_height(e) for e in edges)
        floor = (self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS + self.TUBE_CLEARANCE
                 + self.body_below)
        roof = roof_tube - self.TUBE_RADIUS - self.TUBE_CLEARANCE - self.body_above
        return (min(max(self.tube_cell_height - self.TUBE_CROSS_DROP, floor), roof),
                floor, roof)

    def _tube_back(self, back, centre_offset=None):
        """(along, lateral) of the upright behind the plane, relative to the
        crossing point. Measured where it was seen, from the parameters if not.

        When it was not seen, it is assumed to stand on the CENTRELINE of the
        structure, and where that centreline is comes from the uprights the
        camera actually measured -- not from gap_side. The two disagree
        whenever the cell flown is not the cell the template picked, and this
        is the number the sideways step afterwards is built on, so taking it
        off gap_side is how the aircraft steps back across the centreline
        into the upright it is meant to be dodging.
        """
        if back is None:
            if centre_offset is not None:
                self.get_logger().warning(
                    f"The back upright was not in view; assuming it stands "
                    f"{self.TUBE_BACK_DISTANCE:.2f} m behind the plane, on the centreline "
                    f"of the structure -- {abs(centre_offset):.2f} m to our "
                    f"{'LEFT' if centre_offset >= 0 else 'RIGHT'}, measured "
                    "off the uprights.")
                return self.TUBE_BACK_DISTANCE, centre_offset, True
            self.get_logger().warning(
                f"The back upright was not in view and neither were two "
                f"uprights to take a centreline from; assuming it stands "
                f"{self.TUBE_BACK_DISTANCE:.2f} m behind the plane, on the "
                f"{self.gap_side.upper()} cell's inner edge.")
            sign = 1.0 if self.gap_side == 'left' else -1.0
            return self.TUBE_BACK_DISTANCE, -sign * 0.5 * self.TUBE_SPACING, False
        self.get_logger().warning(
            f"Back upright measured {back['along']:.2f} m behind the plane, "
            f"{back['lateral']:+.2f} m off the track (+ = left).")
        return back['along'], back['lateral'], True

    def _tube_shift_offset(self, back_lateral, measured=False):
        """How far sideways to step after the gap, + LEFT.

        Left by preference, as far as it takes to have half an airframe plus
        the clearance between a prop tip and the back upright, and never less
        than tube_shift_left. Right instead, but only when going left would
        mean an absurd step -- which is what happens when the back upright is
        off to the left of the track already.
        """
        need = self.tube_half_airframe + self.TUBE_CLEARANCE
        least = abs(self.TUBE_SHIFT_LEFT)
        go_left = back_lateral + need
        go_right = back_lateral - need
        if go_left <= self.TUBE_MAX_SHIFT:
            return max(least, go_left)
        if not measured and not self.TUBE_ALLOW_SHIFT_RIGHT:
            # Left is the direction the course is flown in and the one the
            # pilot expects to see. Going right instead because an estimate
            # said the back upright was somewhere odd is how the aircraft ends
            # up on the wrong side of it, so it is off by default: step the
            # most we are allowed to, to the LEFT, and say the measurement
            # looks wrong.
            self.get_logger().error(
                f"The back upright measures {back_lateral:+.2f} m to the LEFT "
                f"of the track, which would take a {go_left:.2f} m step to go "
                f"round on the left (max {self.TUBE_MAX_SHIFT:.2f}). That is "
                "probably a bad measurement -- on this course it stands to the "
                f"right. Stepping {self.TUBE_MAX_SHIFT:.2f} m LEFT anyway; set "
                "tube_allow_shift_right:=true to let it go round the other "
                "side instead.")
            return self.TUBE_MAX_SHIFT
        self.get_logger().warning(
            f"Stepping {abs(go_right):.2f} m RIGHT after the gap: the back "
            f"upright is {back_lateral:+.2f} m to the LEFT of the track and "
            f"going round it on the left would take {go_left:.2f} m. This is "
            "the correct side when the cell crossed was the right-hand one -- "
            "stepping left there walks back across the centreline into it.")
        return min(-least, go_right)

    def tube_exit_distance(self):
        """How far past the PLANE the tubes are done.

        A fixed run-on, not a distance worked out from the back upright: the
        sideways step is what clears that, and this is just how far to carry
        on before calling the obstacle flown. tube_exit_distance > 0 overrides
        it with a distance measured from the shift point, as it used to be.
        """
        if self.TUBE_EXIT_DISTANCE > 0.0:
            return self.TUBE_PASS_EXIT + self.TUBE_EXIT_DISTANCE
        return max(self.TUBE_EXIT_FORWARD, self.TUBE_PASS_EXIT)

    def tube_template_offset(self):
        """Centre of area of the template cell, off the midpoint, + LEFT."""
        sign = 1.0 if self.gap_side == 'left' else -1.0
        return self.tube_cell_lateral - sign * 0.5 * self.TUBE_SPACING

    def _tube_lean(self, hint):
        """How far to aim off the middle of the opening, + LEFT.

        tube_cross_left metres TOWARDS THE HIGH END of the diagonal, not
        towards the aircraft's left. On this obstacle they are the same thing,
        but only because of how it is built and which way it is approached,
        and leaning the wrong way is worse than not leaning: a centimetre of
        lean costs a centimetre of the eight there are to an upright and buys
        a centimetre of headroom under a diagonal that is half a metre clear.

        Measured from the roof at the two shoulders of the cell when the
        camera has it, from whichever cell gap_side picked when it does not.
        """
        if hint is not None and abs(hint['rise']) > 1e-3:
            return math.copysign(self.TUBE_CROSS_LEFT, hint['rise'])
        return math.copysign(self.TUBE_CROSS_LEFT,
                             1.0 if self.gap_side == 'left' else -1.0)

    @property
    def tube_half_airframe(self):
        return 0.5 * self.DRONE_WIDTH + self.TUBE_RADIUS + self.TUBE_LATERAL_MARGIN

    def set_detection(self, which, on):
        """Turn one detector's processing on or off, and say so once."""
        if self.detection_on.get(which) == bool(on):
            return
        self.detection_on[which] = bool(on)
        msg = Bool()
        msg.data = bool(on)
        self.detection_pubs[which].publish(msg)
        self.get_logger().warning(
            f"{which} detection {'ON' if on else 'OFF'}.")

    def tube_detected_callback(self, msg):
        self.tubes_flag = bool(msg.data)

    def tube_geometry_callback(self, msg):
        self.tube_geometry_seen += 1
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0 or data.size % STRIDE != 0:
            return
        lp = self.local_position
        if (lp is None or not lp.xy_valid or not lp.z_valid
                or self.home_z is None or self.attitude is None):
            return
        q = np.asarray(self.attitude.q, dtype=float)
        pos = np.array([lp.x, lp.y, lp.z])
        now = time.monotonic()
        for row in data.reshape(-1, STRIDE):
            self.tube_estimator.add(row, q, pos, self.r_cam, self.t_cam,
                                    self.home_z, now)

    def tube_hole_callback(self, msg):
        """The camera's centre of the biggest cell -> the estimator."""
        data = np.asarray(msg.data, dtype=float)
        if data.size == 0 or data.size % HOLE_STRIDE != 0:
            return
        lp = self.local_position
        if (lp is None or not lp.xy_valid or not lp.z_valid
                or self.home_z is None or self.attitude is None):
            return
        q = np.asarray(self.attitude.q, dtype=float)
        p = np.array([lp.x, lp.y, lp.z])
        now = time.monotonic()
        # A cell the detector could fit a clean quadrilateral to first: that
        # is the trapezium of the obstacle.
        for row in sorted(data.reshape(-1, HOLE_STRIDE), key=lambda r: -r[8]):
            ok, _ = self.tube_estimator.add_hole(row, q, p, self.r_cam,
                                                 self.t_cam, self.home_z, now)
            if ok:
                return

    def _update_tube_solution(self):
        lp = self.local_position
        if lp is None or self.home_z is None:
            self.tube_solution = None
            return
        heading = self.gap_heading if self.gap_heading is not None else self.home_yaw
        u = np.array([math.sin(heading), -math.cos(heading)])
        cells = self.tube_estimator.holes(time.monotonic())
        cell = None
        if not cells and self.scan_choice is not None:
            # Nothing fresh, but the scan chose an opening and the aircraft
            # has not moved since. Fly what the scan found rather than stop
            # for want of a frame -- this is the "go with the best estimate"
            # case, and the crossing is committed from a frozen path anyway.
            cells = [self.scan_choice]
        if cells:
            if self.scan_choice is not None:
                # Stay on the opening the scan picked. Nearest cluster to
                # where it was, not whichever one happens to measure biggest
                # from here -- that is the decision that was already made, and
                # remaking it every frame from a worse viewpoint is how the
                # aircraft ends up committed to the other cell.
                near = min(cells, key=lambda c: float(
                    np.linalg.norm(c['xy'] - self.scan_choice['xy'])))
                if float(np.linalg.norm(near['xy'] - self.scan_choice['xy'])) \
                        <= self.TUBE_SPACING * 0.75:
                    cell = near
            if cell is None:
                cell = cells[0]
        hint = None
        if cell is not None:
            hint = {'lateral': float(np.dot(cell['xy'] - np.array([lp.x, lp.y]), u)),
                    'height': cell['height'], 'ceiling': cell['ceiling'],
                    'floor': cell['floor'], 'width': cell['width'],
                    'rise': cell['rise'], 'count': cell['count'],
                    'cut': cell.get('cut', False)}
        clusters = self.tube_estimator.clusters(time.monotonic())
        sol, reason = solve_gap(
            clusters, (lp.x, lp.y), heading,
            self.TUBE_SPACING, self.TUBE_PLANE_BAND, self.TUBE_MATCH_TOLERANCE,
            self.TUBE_MIN_MATCHED, self.TUBE_MAX_PLANE_YAW, self.gap_side,
            prefer_lateral=(None if hint is None
                            else hint['lateral'] + self._tube_lean(hint)),
            lateral_offset=self.tube_template_offset() + self._tube_lean(None),
            half_airframe=self.tube_half_airframe)
        if sol is not None and not (self.TUBE_MIN_GAP_WIDTH <= sol['width']
                                    <= self.TUBE_MAX_GAP_WIDTH):
            sol, reason = None, f"measured gap {sol['width']:.2f} m is implausible"

        if sol is None and cell is not None:
            # No layout fit, but the opening itself has been measured. Fly
            # THAT. The three-upright template is a guess at where the opening
            # is; the opening is the opening. This is what gets flown when the
            # obstacle is not three evenly spaced tubes -- two of them in
            # view, a gate built differently, a third upright out of frame --
            # and refusing it means hovering in front of a hole the camera can
            # see perfectly well.
            sol, reason = self._tube_solve_from_cell(cell, clusters, heading,
                                                     (lp.x, lp.y), reason)
        if sol is not None:
            sol['hole'] = hint
        self.tube_solution = sol
        self.tube_reason = reason
        if sol is None:
            self.tube_ok_since = None
        elif self.tube_ok_since is None:
            self.tube_ok_since = time.monotonic()

    def _tube_solve_from_cell(self, cell, clusters, heading, vehicle_xy, why):
        """A crossing from the measured opening, when the layout will not fit."""
        fits_wide = cell['width'] >= self.DRONE_WIDTH + 2.0 * self.TUBE_LATERAL_MARGIN
        tall = cell['ceiling'] - cell['floor']
        fits_tall = tall >= self.DRONE_HEIGHT + 2.0 * self.TUBE_CLEARANCE
        if not (fits_wide and fits_tall):
            return None, (f"{why}; and the measured opening ({cell['width']:.2f} "
                          f"x {tall:.2f} m) is too small for the airframe")
        plane, plane_why = front_plane(clusters, vehicle_xy, heading,
                                       self.TUBE_PLANE_BAND, self.TUBE_MAX_PLANE_YAW)
        if plane is None:
            # No uprights either -- they can all be rejected while the OPENING
            # is still being measured perfectly well, because a cell is
            # bounded by tubes the camera sees clearly while the same tubes'
            # lower ends are lost against the floor. The opening alone is
            # enough: the obstacle stands square to the course, which is the
            # line the aircraft has been flying since the window, so the
            # course heading stands in for the plane and the distance to it
            # comes from the measured cell.
            v = np.asarray(vehicle_xy, dtype=float)
            fwd = np.array([math.cos(heading), math.sin(heading)])
            plane = {'front': np.empty((0, 2)),
                     'u': np.array([math.sin(heading), -math.cos(heading)]),
                     'normal': fwd, 'heading': heading,
                     'along': float(np.dot(np.asarray(cell['xy']) - v, fwd)),
                     'behind': []}
            self.get_logger().warning(
                f"No tube plane ({plane_why}), so squaring the crossing to the "
                "COURSE heading and taking the range from the opening itself. "
                "The uprights are being rejected; the opening is not.",
                throttle_duration_sec=5.0)
        sol = solve_from_hole(plane, vehicle_xy, cell['xy'], cell['width'],
                              self.tube_half_airframe,
                              lean=self._tube_lean({'rise': cell['rise']}))
        self.get_logger().warning(
            "Flying the MEASURED opening, not the template: %s. %.2f x %.2f m, "
            "%d uprights in the plane." % (why, cell['width'], tall,
                                           len(plane['front'])),
            throttle_duration_sec=5.0)
        return sol, ''

    def tube_summary(self):
        s = self.tube_solution
        if s is None:
            if self.tube_geometry_seen == 0:
                return "nothing on /tube_geometry -- is tube_detect running?"
            return (f"no gap: {self.tube_reason} "
                    f"({self.tube_estimator.accepted_total} samples accepted; "
                    f"rejections: {self.tube_estimator.rejection_summary()})")
        h = s.get('hole')
        hole = (f"hole centre {s['offset']:+.2f} m off the midpoint at "
                f"{h['height']:.2f} m, ceiling {h['ceiling']:.2f}"
                f"{' (frame, roof is higher)' if h.get('cut') else ''}, floor "
                f"{h['floor']:.2f}, roof rising {h['rise']:+.2f} m/m left"
                if h is not None
                else "no hole in view; aiming at the template centre")
        return (f"{s['side'].upper()} gap at ({s['point'][0]:+.2f}, "
                f"{s['point'][1]:+.2f}), {s['width']:.2f} m wide, "
                f"{s['matched']} uprights matched, {hole}")

    def publish_tube_gap(self):
        msg = String()
        s = self.tube_solution
        msg.data = '' if s is None else "|".join([
            f"{s['point'][0]:.3f}", f"{s['point'][1]:.3f}",
            f"{math.degrees(s['heading']):.1f}", f"{s['width']:.3f}",
            f"{s['matched']}"])
        self.tube_gap_pub.publish(msg)

    def _freeze_tube_path(self, sol):
        self.gap_point = np.array(sol['point'], dtype=float)
        self.gap_normal = np.array(sol['normal'], dtype=float)
        self.gap_left = np.array(sol['left'], dtype=float)
        self.gap_heading = sol['heading']
        self.tube_entry = self.gap_point - self.gap_normal * self.TUBE_STANDOFF
        self.tube_pass_exit = self.gap_point + self.gap_normal * self.TUBE_PASS_EXIT
        self.tube_back_along, self.tube_back_lateral, back_measured = self._tube_back(
            sol.get('back'), sol.get('centre_offset'))
        self.tube_shift = self._tube_shift_offset(self.tube_back_lateral,
                                                  back_measured)
        self.tube_shift_point = self.tube_pass_exit + self.gap_left * self.tube_shift
        self.tube_final_point = (self.gap_point
                                 + self.gap_normal * self.tube_exit_distance()
                                 + self.gap_left * self.tube_shift)
        self.tube_altitude = self._tube_crossing_altitude(sol.get('hole'))
        self._log_tube_plan(sol)

    def _log_tube_plan(self, sol):
        """Every direction of the crossing, in words, before it is flown.

        Signs in this file are all "+ is the aircraft's left", which is easy
        to state and easy to get backwards somewhere in the chain. This prints
        what the aircraft is actually about to do, so a reversed direction
        shows up in the log before it shows up in the wreckage.
        """
        hole = sol.get('hole')
        side = lambda v: 'LEFT' if v >= 0 else 'RIGHT'
        if sol.get('source') == 'hole':
            where = ("the MEASURED opening, %.2f m wide, aiming %.2f m to its "
                     "%s of centre" % (sol['width'], abs(sol['offset']),
                                       side(sol['offset'])))
        else:
            where = ("the %s cell, %.2f m to the %s of the middle upright"
                     % (self.gap_side.upper(),
                        abs(sol['offset'] + 0.5 * self.TUBE_SPACING),
                        side(1.0 if self.gap_side == 'left' else -1.0)))
        self.get_logger().warning(
            "TUBE PLAN: cross %s, at %.2f m. %s the roof: %s. Then %.2f m to "
            "the %s and %.2f m on, passing the back upright %.2f m to our %s."
            % (where, self.tube_altitude,
               'MEASURED' if hole is not None else 'TEMPLATE',
               (f"rises {abs(hole['rise']):.2f} m/m to the {side(hole['rise'])}"
                if hole is not None else
                f"assumed high on the {self.gap_side.upper()}"),
               abs(self.tube_shift), side(self.tube_shift),
               self.tube_exit_distance(),
               abs(self.tube_back_lateral), side(self.tube_back_lateral)))

    def _tube_crossing_altitude(self, hole):
        """The crossing height, from the MEASURED ceiling and floor of the cell.

        Not from the template band: that is built from diagonal_left_height
        and diagonal_right_height, and if those are the wrong way round -- the
        obstacle built mirrored, or the aircraft coming at it from the other
        side -- the band says there is room exactly where the diagonal is. The
        camera measures the ceiling over the aircraft's own track, so where
        the two disagree the camera wins and the disagreement is logged.
        """
        if hole is None:
            return self.tube_altitude
        lo = hole['floor'] + self.TUBE_CLEARANCE + self.body_below
        hi = hole['ceiling'] - self.TUBE_CLEARANCE - self.body_above
        wanted = hole['height'] - self.TUBE_CROSS_DROP
        if hi < lo:
            altitude = 0.5 * (hole['floor'] + hole['ceiling'])
            self.get_logger().error(
                f"The measured cell ({hole['floor']:.2f}-{hole['ceiling']:.2f} m) "
                f"is under {self.DRONE_HEIGHT + 2 * self.TUBE_CLEARANCE:.2f} m "
                f"tall. Threading the middle of it at {altitude:.2f} m.")
        else:
            altitude = min(max(wanted, lo), hi)
        altitude = min(max(altitude, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if (not hole.get('cut')
                and not (self.tube_floor - 0.05 <= altitude <= self.tube_roof + 0.05)):
            self.get_logger().error(
                f"MEASURED crossing height {altitude:.2f} m is outside the "
                f"TEMPLATE band {self.tube_floor:.2f}-{self.tube_roof:.2f} m. "
                "Trusting the camera, but check cross_bar_height and "
                "diagonal_left_height/diagonal_right_height -- left and right "
                "are as the APPROACHING AIRCRAFT sees them, and having them "
                "the wrong way round is what flies it into the diagonal.")
        self.get_logger().warning(
            f"Crossing the tubes at {altitude:.2f} m: the camera puts the cell "
            f"at {hole['floor']:.2f}-{hole['ceiling']:.2f} m over the track, "
            f"centre of area {hole['height']:.2f} m (template said "
            f"{self.tube_altitude:.2f} m)."
            + (" The ceiling is the top of the FRAME, not the diagonal, so the "
               "real cell is taller than this and the height flown is low in "
               "it -- which is the safe way round." if hole.get('cut') else ""))
        return altitude

    def _tube_frame(self, target):
        lp = self.local_position
        if lp is None or target is None or self.gap_normal is None:
            return None, None
        e = np.asarray(target) - np.array([lp.x, lp.y])
        return float(np.dot(e, self.gap_normal)), float(np.dot(e, self.gap_left))

    def _tube_past_plane(self):
        lp = self.local_position
        if lp is None or self.gap_point is None:
            return -1e9
        return float(np.dot(np.array([lp.x, lp.y]) - self.gap_point, self.gap_normal))

    def _tube_arrived(self, target):
        along, cross = self._tube_frame(target)
        if (along is not None and abs(along) <= self.TUBE_ARRIVE_TOLERANCE
                and abs(cross) <= self.TUBE_ARRIVE_TOLERANCE):
            now = time.monotonic()
            if self.tube_settle_since is None:
                self.tube_settle_since = now
            return now - self.tube_settle_since >= 0.5
        self.tube_settle_since = None
        return False

    def _enter_tube_stage(self, stage):
        self._enter_stage(stage)
        self.tube_settle_since = None
        self.tube_push_since = None
        self.course_flow_lost_since = None

    def _tube_look_altitude(self):
        """Where to sit while scanning: asked for, or clear of the blue bar.

        The scan happens a short way in front of the second blue bar, and
        getting there means backing UP towards it. At 1.98 m in the scaled
        world -- which is what tube_look_altitude 0.90 came to -- the belly
        ended up 16 mm above the top of that bar. So the asked-for altitude is
        a floor, not the answer: if it does not leave tube_clearance between
        the airframe and the bar, it is raised until it does.
        """
        over_blue = (self.BLUE_BAR_HEIGHT + self.BAR_RADIUS
                     + self.TUBE_CLEARANCE + self.body_below)
        if over_blue <= self.TUBE_LOOK_ALTITUDE:
            return self.TUBE_LOOK_ALTITUDE
        gap = (self.TUBE_LOOK_ALTITUDE - self.body_below
               - self.BLUE_BAR_HEIGHT - self.BAR_RADIUS)
        if not self.TUBE_RAISE_OVER_BLUE:
            # Advisory, not automatic, and deliberately so. The scan altitude
            # is the single number this whole stage is most sensitive to:
            # raising it moves the camera's horizon down the frame, and an
            # upright whose lower end is then lost against the floor is
            # rejected as "does not reach the floor" -- which is how a working
            # scan stopped working. Changing it silently, in the name of a
            # clearance the aircraft never actually flies into, cost two
            # flights. So it says the number and leaves the geometry alone.
            self.get_logger().warning(
                f"Scan altitude {self.TUBE_LOOK_ALTITUDE:.2f} m leaves "
                f"{gap:+.2f} m between the airframe and the blue bar at "
                f"{self.BLUE_BAR_HEIGHT:.2f} m ({over_blue:.2f} m would give "
                f"the full {self.TUBE_CLEARANCE:.2f} m). The aircraft stops "
                "in FRONT of that bar, not over it. tube_raise_over_blue:=true "
                "to raise it anyway -- and then check that the uprights are "
                "still being accepted.")
            return self.TUBE_LOOK_ALTITUDE
        self.get_logger().warning(
            f"Scan altitude raised from {self.TUBE_LOOK_ALTITUDE:.2f} to "
            f"{over_blue:.2f} m for the blue bar (tube_raise_over_blue).")
        return over_blue

    def _tube_look_move(self):
        """The look-from move, capped so the aircraft does not back over the bar."""
        move = self._tube_look_move_uncapped()
        if move >= 0.0 or self.TUBE_BACK_OFF_MAX <= 0.0:
            return move
        if -move <= self.TUBE_BACK_OFF_MAX:
            return move
        self.get_logger().warning(
            f"TUBE: backing up {abs(move):.2f} m would put the aircraft over "
            f"the blue bar it has just flown under -- a {self.BLUE_BAR_HEIGHT:.2f} m "
            "step in the lidar, which is what drops EKF2's rangefinder "
            f"fusion. Capped at {self.TUBE_BACK_OFF_MAX:.2f} m "
            "(tube_back_off_max); the scan makes up the difference by "
            "sweeping the yaw and, if that sees nothing, by trying "
            f"{self.TUBE_SCAN_ALT_STEPS} scan altitudes "
            f"{self.TUBE_SCAN_ALT_STEP:.2f} m apart.")
        return -self.TUBE_BACK_OFF_MAX

    def _tube_look_move_uncapped(self):
        """How far to move to reach the look-from point, + being forward.

        Reckoned from the OBSTACLE, not from wherever the blue crossing
        happened to stop. The blue crossing ends blue_exit past the second
        blue bar, the tube plane is tube_plane_from_blue past that same bar,
        so what is left to the plane is the difference -- and the look-from
        point is tube_look_standoff short of it. On this course that comes out
        NEGATIVE: the blue crossing already leaves the aircraft closer to the
        tubes than it wants to be, and the right move is backwards.

        Getting this wrong is not a tuning matter. Measured the old way the
        move came out 3.30 m in the scaled world, which put the look-from
        point 1.76 m PAST the obstacle and flew the aircraft into it while it
        was still climbing.
        """
        if self.BLUE_TO_TUBE > 0.0:
            return self.BLUE_TO_TUBE
        move = self.TUBE_PLANE_FROM_BLUE - self.BLUE_EXIT - self.TUBE_LOOK_STANDOFF
        if move >= 0.0 or self._tube_clears_blue():
            # Free to back up as far as the standoff wants, over the blue bar
            # if it comes to that: the scan altitude clears it.
            return move
        # It does not clear the bar, so backing up towards it is limited by
        # the bar rather than by the standoff.
        most_back = max(0.0, self.BLUE_EXIT - self.TUBE_BLUE_CLEAR)
        if move < -most_back:
            self.get_logger().error(
                f"Look-from point capped at {most_back:.2f} m back instead of "
                f"{abs(move):.2f}: the scan altitude does not clear the blue "
                f"bar. That leaves only "
                f"{self.TUBE_LOOK_STANDOFF + move + most_back:.2f} m to the "
                "tube plane, which may be too close to see the whole opening "
                "-- raise tube_look_altitude instead of accepting this.")
            return -most_back
        return move

    def _tube_clears_blue(self):
        """True if the airframe at the scan altitude passes over the blue bar."""
        return (self.TUBE_LOOK_ALTITUDE - self.body_below
                >= self.BLUE_BAR_HEIGHT + self.BAR_RADIUS + self.TUBE_CLEARANCE)

    def _begin_tube_climb(self):
        """Climb to the look-from altitude where we are -- no forward move.

        Climbing and closing on the obstacle at the same time is what turned a
        bad look-from point into a collision. The climb happens on the spot,
        clear of everything, and only then does the aircraft reposition.
        """
        lp = self.local_position
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(lp.x, lp.y, self.TUBE_LOOK_ALTITUDE)
        self._enter_tube_stage(self.TUBE_CLIMB)
        self.tube_scan_alt_tries = 0
        self.set_detection('tube', True)
        self.get_logger().warning(
            f"TUBE_CLIMB: past the blue bars. Straight up to "
            f"{self.TUBE_LOOK_ALTITUDE:.2f} m where we are, then "
            f"{self._tube_look_move():+.2f} m to sit "
            f"{self.TUBE_LOOK_STANDOFF:.2f} m short of the tube plane and "
            f"sweep +/-{math.degrees(self.TUBE_SCAN_YAW):.0f} deg across it.")

    def _handle_tube_climb(self):
        if self._flow_lost_for() > self.COURSE_FLOW_TIMEOUT:
            self._hold_and_wait(self.TUBE_CLIMB, "flow lost on the way to the tubes")
            return
        along, _ = self._target_errors()
        alt = self.relative_altitude()
        settled = (along is not None and abs(along) <= self.COURSE_XY_TOLERANCE
                   and alt is not None
                   and abs(alt - self.commanded_altitude) <= self.COURSE_ALT_TOLERANCE)
        if settled or self._in_stage_for() > self.COURSE_VERTICAL_TIMEOUT:
            self._begin_tube_approach(settled)
            return
        self.get_logger().info(
            f"TUBE_CLIMB: {0.0 if along is None else along:+.2f} m to go, alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.commanded_altitude:.2f} m.",
            throttle_duration_sec=1.0)

    def _begin_tube_approach(self, settled):
        """Reposition to the look-from point. Forwards or backwards."""
        move = self._tube_look_move()
        # Where the gate plane is, reckoned from the course, relative to the
        # point we are about to fly to. Only the blind crossing uses it, and
        # only when the camera has given it nothing better.
        self.tube_plane_ahead = (self.TUBE_PLANE_FROM_BLUE - self.BLUE_EXIT
                                 - move) if self.BLUE_TO_TUBE <= 0.0 else (
            self.TUBE_LOOK_STANDOFF)
        x, y = self._ahead(move)
        self.MOVE_SPEED = self.PAD_SEARCH_SPEED if move < 0.0 else self.APPROACH_SPEED
        self._set_target(x, y, self.TUBE_LOOK_ALTITUDE)
        self._enter_tube_stage(self.TUBE_APPROACH)
        self.get_logger().warning(
            "TUBE_APPROACH: %s%+.2f m to the look-from point, %.2f m short of "
            "the tube plane. Stopping early if an upright comes up closer "
            "than that." % ("" if settled else "(climb did not settle) ",
                            move, self.TUBE_LOOK_STANDOFF))

    def _nearest_upright(self):
        """Distance to the closest upright the camera has, or None."""
        lp = self.local_position
        if lp is None:
            return None
        clusters = self.tube_estimator.clusters(time.monotonic())
        fwd = np.array([math.cos(self._course_heading()),
                        math.sin(self._course_heading())])
        ahead = [float(np.dot(xy - np.array([lp.x, lp.y]), fwd))
                 for xy, _ in clusters]
        ahead = [a for a in ahead if a > 0.0]
        return min(ahead) if ahead else None

    def _handle_tube_approach(self):
        self._aim_yaw_at(self.traverse_heading)
        if not self.hold_xy:
            self._hold_and_wait(self.TUBE_APPROACH, "flow lost before the tubes")
            return

        along, cross = self._target_errors()

        # The camera has the last word on how close is close enough. The
        # look-from move is worked out from course measurements, and course
        # measurements are exactly what was wrong when this flew into the
        # obstacle; an upright standing closer than the standoff stops the
        # approach wherever it is.
        #
        # Only while CLOSING, though. The move is often backwards -- the blue
        # crossing tends to end closer to the tubes than the scan wants to be
        # -- and an upright inside the standoff is the whole reason for
        # backing up. Stopping on it then would cancel the very move that
        # fixes it.
        near = self._nearest_upright()
        if (along is not None and along > 0.0
                and near is not None and near <= self.TUBE_LOOK_STANDOFF):
            self.moving = False
            self.get_logger().warning(
                f"TUBE_APPROACH: an upright is {near:.2f} m ahead, inside the "
                f"{self.TUBE_LOOK_STANDOFF:.2f} m standoff. Stopping here and "
                "scanning from here.")
            self._begin_tube_scan(True)
            return

        if (along is not None and abs(along) <= self.COURSE_XY_TOLERANCE
                and abs(cross) <= self.COURSE_XY_TOLERANCE):
            self._begin_tube_scan(True)
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._begin_tube_scan(False)
            return
        self.get_logger().info(
            f"TUBE_APPROACH: {0.0 if along is None else along:+.2f} m to go, "
            f"nearest upright {'n/a' if near is None else f'{near:.2f} m'}.",
            throttle_duration_sec=1.0)

    def _begin_tube_scan(self, settled):
        """Stand still and sweep the yaw across the obstacle before deciding.

        From one heading the camera sees one cell well and the other at an
        angle, and the one it sees well is the one that measures biggest --
        which is how the aircraft ends up committed to the small side. A sweep
        puts both of them in front of the camera squarely, at the same
        distance, and the estimator holds them as two separate clusters in
        NED, so what picks the gap is the measured size of the opening rather
        than which way the nose happened to be pointing.
        """
        self.moving = False
        self.tube_estimator.hole_samples.clear()
        self.scan_since = time.monotonic()
        self.scan_ended = None
        self.scan_report = None
        self._enter_tube_stage(self.TUBE_SCAN)
        self.get_logger().warning(
            "TUBE_SCAN: %s Holding still and sweeping +/-%.0f deg for %.0f s "
            "to see both openings."
            % ("On the look-from point." if settled else
               "Did not settle on the look-from point; scanning from here.",
               math.degrees(self.TUBE_SCAN_YAW), self.TUBE_SCAN_SECONDS))

    def _scan_yaw_now(self):
        """Where the nose should be pointing, this far into the sweep.

        One pass per tube_scan_seconds -- centre, left, right, centre -- and
        it keeps going round if the sweep is extended, so an extension is just
        more of the same rather than a new kind of motion.
        """
        t = (time.monotonic() - self.scan_since) / max(self.TUBE_SCAN_SECONDS, 1e-3)
        return wrap_pi(self.traverse_heading
                       + self.TUBE_SCAN_YAW * math.sin(2.0 * math.pi * t))

    def _handle_tube_scan(self):
        self.moving = False
        if not self.hold_xy:
            # Sweeping the nose on a bad lateral estimate drags the aircraft
            # sideways with it. Hold, then start the sweep again.
            self._hold_and_wait(self.TUBE_CLIMB, "flow lost during the scan")
            return
        elapsed = time.monotonic() - self.scan_since
        seen = bool(self.tube_estimator.holes(
            time.monotonic(), buffer_seconds=self.TUBE_SCAN_MAX_SECONDS + 10.0))

        # Keep sweeping past the nominal time if NOTHING has been seen yet.
        # One pass is plenty when the obstacle is in view the whole way round;
        # when it is not -- a post leaving the frame at the ends of the sweep,
        # a mask that only closes now and then -- a few more passes cost
        # seconds and save the whole stage. Once something has been seen, the
        # sweep ends on time and the best of it is flown.
        keep_sweeping = (elapsed < self.TUBE_SCAN_SECONDS
                         or (not seen and elapsed < self.TUBE_SCAN_MAX_SECONDS))
        if keep_sweeping:
            self._aim_yaw_at(self._scan_yaw_now())
            self.get_logger().info(
                "TUBE_SCAN: %.1f/%.0f s%s, %s"
                % (elapsed, self.TUBE_SCAN_SECONDS,
                   "" if seen else f" (extending to {self.TUBE_SCAN_MAX_SECONDS:.0f} s,"
                                   " nothing seen yet)",
                   self._scan_summary()), throttle_duration_sec=1.0)
            return
        if not seen and self._retry_scan_from_another_altitude():
            return
        if not seen:
            self.get_logger().error(
                f"TUBE_SCAN: {elapsed:.0f} s of sweeping and no opening ever "
                "came out of the mask. Going on to search from here -- the "
                "crossing can still be built from an opening seen later.")

        # Sweep done: square up, let the estimate settle, then choose.
        if self.scan_ended is None:
            self.scan_ended = time.monotonic()
        self._aim_yaw_at(self.traverse_heading)
        if (time.monotonic() - self.scan_ended < self.TUBE_SCAN_SETTLE
                or self._heading_error(self.traverse_heading) > self.TUBE_YAW_TOLERANCE):
            self.get_logger().info(f"TUBE_SCAN: squaring up. {self._scan_summary()}",
                                   throttle_duration_sec=1.0)
            if time.monotonic() - self.scan_ended > self.TUBE_STAGE_TIMEOUT:
                self.get_logger().error("TUBE_SCAN: never squared up; carrying on.")
            else:
                return

        chosen = self._choose_gap()
        if chosen is None:
            self._enter_tube_stage(self.TUBE_SEARCH)
            self.get_logger().error(
                f"TUBE_SCAN: saw no opening in the sweep ({self._scan_summary()}). "
                "Falling back to searching from here.")
            return
        self.scan_choice = chosen
        self._enter_tube_stage(self.TUBE_SEARCH)

    def _retry_scan_from_another_altitude(self):
        """Move the scan altitude and sweep again. True if a retry started.

        A sweep that saw nothing at all is usually a FRAMING problem, not a
        detection one: from the capped look-from point the gate is close, and
        at one height the uprights run off the top of the frame while at
        another their feet are lost against the floor. Yaw alone cannot fix
        that -- it is the wrong axis -- so the altitude is stepped instead,
        up first (which lifts the horizon and keeps the feet in frame) and
        then down, and the whole sweep is flown again from there.

        The aircraft is stationary throughout, well short of the plane, so
        the only thing moving is its height.
        """
        if self.tube_scan_alt_tries >= self.TUBE_SCAN_ALT_STEPS:
            return False
        lp = self.local_position
        if lp is None:
            return False
        self.tube_scan_alt_tries += 1
        # +1, -1, +2, -2 ... about the altitude the stage started from.
        n = (self.tube_scan_alt_tries + 1) // 2
        sign = 1.0 if self.tube_scan_alt_tries % 2 else -1.0
        wanted = self.TUBE_LOOK_ALTITUDE + sign * n * self.TUBE_SCAN_ALT_STEP
        altitude = min(max(wanted, self.MIN_ALTITUDE), self.MAX_ALTITUDE)
        if abs(altitude - self.commanded_altitude) < 1e-3:
            return False
        self._set_target(lp.x, lp.y, altitude)
        self.get_logger().error(
            f"TUBE_SCAN: a whole sweep and nothing came out of the mask. "
            f"Retry {self.tube_scan_alt_tries} of {self.TUBE_SCAN_ALT_STEPS}: "
            f"moving the scan altitude {altitude - self.commanded_altitude:+.2f} m "
            f"to {altitude:.2f} m and sweeping again -- from this close the "
            "whole obstacle does not fit the frame at every height.")
        self._begin_tube_scan(True)
        return True

    # ------------------------------------------- the blind tube crossing

    def _blind_plane_ahead(self):
        """How far ahead the gate plane is, measured if possible."""
        near = self._nearest_upright()
        if near is not None:
            return near
        along, _ = self._target_errors()
        nominal = (self.TUBE_LOOK_STANDOFF if self.tube_plane_ahead is None
                   else self.tube_plane_ahead)
        # tube_plane_ahead is measured from the look-from TARGET; anything
        # still to go to that target is still to go to the plane as well.
        return max(0.3, nominal + (0.0 if along is None else along))

    def _tube_gave_up(self, why):
        """A tube stage has run out of camera. Blind crossing, or land."""
        if self.TUBE_BLIND and not self.tube_blind_flown and self.hold_xy:
            self._begin_tube_blind(why)
            return
        self._abandon(why)

    def _begin_tube_blind(self, why):
        """Cross on the KNOWN geometry when nothing ever solved.

        The camera has had its yaw sweep, its altitude retries and its search,
        and has produced nothing that solve_gap or solve_from_hole would
        accept. The alternative to this is landing in front of the gate, and
        the geometry is not actually unknown: the gate stands square across
        the course line, its uprights are tube_spacing apart, and the cell the
        course is flown through is the one between the middle upright and the
        upright on tube_gap_prefer's side -- which is the side the course
        leaves on, and the side the lone back upright is NOT on.

        So: step half a spacing to that side of the track, and fly through at
        the template altitude. Everything after the crossing -- the sideways
        step round the back upright and the run-on -- is the normal path,
        because it never depended on the camera either.

        It is flown once. If the aircraft is not through after that, something
        is wrong that another blind attempt will not fix.
        """
        lp = self.local_position
        if lp is None:
            self._abandon(f"{why}, and there is no position to cross blind on")
            return
        self.tube_blind_flown = True
        h = self.traverse_heading
        fwd = np.array([math.cos(h), math.sin(h)])
        left = np.array([math.sin(h), -math.cos(h)])
        v = np.array([lp.x, lp.y])

        side = self.TUBE_GAP_PREFER if self.TUBE_GAP_PREFER != 'none' else 'left'
        self.gap_side = side
        sign = 1.0 if side == 'left' else -1.0
        lateral = sign * 0.5 * self.TUBE_SPACING
        ahead = self._blind_plane_ahead()

        sol = {
            'point': v + fwd * ahead + left * lateral,
            'normal': fwd,
            'left': left,
            'heading': h,
            'width': self.TUBE_SPACING,
            'offset': 0.0,
            # The structure's middle is the upright on the track, which is
            # half a spacing back the other way from where we are crossing.
            'centre_offset': -lateral,
            'side': side,
            'back': None,
            'hole': None,
            'matched': 0,
            'residual': 0.0,
            'source': 'blind',
        }
        self.tube_solution = sol
        self.tube_reason = 'blind crossing on the known geometry'
        self.get_logger().error(
            f"TUBE BLIND CROSSING: {why}. Nothing the camera produced ever "
            f"solved, so the gate is being flown on its KNOWN geometry: the "
            f"plane {ahead:.2f} m ahead, {abs(lateral):.2f} m to our "
            f"{side.upper()} of the track -- between the middle upright and "
            f"the {side} one -- at {self.tube_altitude:.2f} m. This is the "
            "alternative to landing in front of it; set tube_blind:=false to "
            "land instead.")
        self._begin_tube_align(sol)

    def _scan_summary(self):
        cells = self.tube_estimator.holes(
            time.monotonic(), buffer_seconds=self.TUBE_SCAN_MAX_SECONDS + 10.0)
        if not cells:
            return "no opening yet"
        return "; ".join(
            f"{c['area']:.2f} m2 ({c['width']:.2f}x{c['ceiling'] - c['floor']:.2f}) "
            f"at ({c['xy'][0]:+.2f},{c['xy'][1]:+.2f}) x{c['count']}"
            for c in cells[:3])

    def _choose_gap(self):
        """The opening to fly, out of everything the sweep saw.

        Biggest measured area wins. Where two are within tube_gap_tie of each
        other -- the two cells of one obstacle often are, seen from far enough
        away -- the tie goes to tube_gap_prefer, which is the LEFT, because
        that is the side the course leaves on and the side the back upright is
        not on.
        """
        now = time.monotonic()
        cells = self.tube_estimator.holes(
            now, buffer_seconds=self.TUBE_SCAN_MAX_SECONDS + 10.0)
        if not cells:
            return None
        lp = self.local_position
        u = np.array([math.sin(self.traverse_heading), -math.cos(self.traverse_heading)])
        for c in cells:
            c['lateral'] = float(np.dot(c['xy'] - np.array([lp.x, lp.y]), u))

        best = cells[0]
        tied = [c for c in cells if c['area'] >= best['area'] * (1.0 - self.TUBE_GAP_TIE)]
        if len(tied) > 1 and self.TUBE_GAP_PREFER != 'none':
            want_left = self.TUBE_GAP_PREFER == 'left'
            best = max(tied, key=lambda c: c['lateral'] if want_left else -c['lateral'])

        for c in cells:
            self.get_logger().warning(
                "TUBE_SCAN saw: %.2f m2%s opening, %.2f m wide, %.2f m tall, "
                "%.2f m to our %s, roof rising %.2f m/m to the %s, %d frames%s"
                % (c['area'], " (at least)" if c.get('cut') else "",
                   c['width'], c['ceiling'] - c['floor'],
                   abs(c['lateral']), 'LEFT' if c['lateral'] >= 0 else 'RIGHT',
                   abs(c['rise']), 'LEFT' if c['rise'] >= 0 else 'RIGHT',
                   c['count'], "   <== CHOSEN" if c is best else ""))
        self.get_logger().warning(
            "TUBE_SCAN: flying the opening %.2f m to our %s (%.2f m2)%s."
            % (abs(best['lateral']), 'LEFT' if best['lateral'] >= 0 else 'RIGHT',
               best['area'],
               f", the {self.TUBE_GAP_PREFER} one of {len(tied)} within "
               f"{self.TUBE_GAP_TIE * 100:.0f}%" if len(tied) > 1 else ""))
        return best

    def _handle_tube_search(self):
        if self.tube_solution is not None and self.hold_xy:
            self._enter_tube_stage(self.TUBE_LOCK)
            self.get_logger().warning(f"TUBE_LOCK: {self.tube_summary()}.")
            return
        if self._in_stage_for() > self.TUBE_SEARCH_TIMEOUT:
            why = (f"no tube gap found in {self.TUBE_SEARCH_TIMEOUT:.0f} s. "
                   f"{self.tube_summary()}")
            if self.TUBE_BLIND and not self.tube_blind_flown and self.hold_xy:
                self._begin_tube_blind(why)
                return
            self._abandon(why)
            return
        self.get_logger().info(f"TUBE_SEARCH: {self.tube_summary()}",
                               throttle_duration_sec=1.0)

    def _handle_tube_lock(self):
        if self.tube_solution is None:
            if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
                self._tube_gave_up(f"lost the tube gap. {self.tube_summary()}")
            return
        if (self.tube_ok_since is None
                or time.monotonic() - self.tube_ok_since < self.TUBE_LOCK_SECONDS):
            self.get_logger().info(f"TUBE_LOCK: settling. {self.tube_summary()}",
                                   throttle_duration_sec=1.0)
            return
        self._begin_tube_align(self.tube_solution)

    def _begin_tube_align(self, sol):
        self._freeze_tube_path(sol)
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_tube_stage(self.TUBE_ALIGN)
        self._set_target(self.tube_entry[0], self.tube_entry[1], self.tube_altitude)
        self._aim_yaw_at(self.gap_heading)
        self.get_logger().warning(
            f"TUBE_ALIGN: gap at ({self.gap_point[0]:+.2f}, {self.gap_point[1]:+.2f}), "
            f"heading {math.degrees(self.gap_heading):+.0f} deg. Entry "
            f"{self.TUBE_STANDOFF:.2f} m short of it at {self.tube_altitude:.2f} m.")

    def _handle_tube_align(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self.moving = False
            self.tube_settle_since = None
            if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
                self._hold_and_wait(self.TUBE_ALIGN, "flow lost before the gap")
            return
        self.moving = True

        along, cross = self._tube_frame(self.tube_entry)
        alt = self.relative_altitude()
        ready = (along is not None
                 and abs(along) <= self.TUBE_ALONG_TOLERANCE
                 and abs(cross) <= self.TUBE_CROSS_TOLERANCE
                 and alt is not None
                 and abs(alt - self.tube_altitude) <= self.TUBE_ALT_TOLERANCE
                 and self._heading_error(self.gap_heading) <= self.TUBE_YAW_TOLERANCE)
        if ready:
            now = time.monotonic()
            if self.tube_settle_since is None:
                self.tube_settle_since = now
            elif now - self.tube_settle_since >= self.TUBE_SETTLE_SECONDS:
                self._begin_tube_pass()
            return
        self.tube_settle_since = None
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._tube_gave_up("could not settle on the gap entry in "
                               f"{self.TUBE_STAGE_TIMEOUT:.0f} s")
            return
        self.get_logger().info(
            f"TUBE_ALIGN: {0.0 if along is None else along:+.2f} along / "
            f"{0.0 if cross is None else cross:+.2f} across (tol "
            f"{self.TUBE_CROSS_TOLERANCE:.2f}), alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.tube_altitude:.2f}. "
            f"{self.tube_summary()}", throttle_duration_sec=1.0)

    def _begin_tube_pass(self):
        if self.tube_solution is None or self.tube_solution.get('hole') is None:
            self.get_logger().error(
                "COMMITTING WITHOUT HAVING SEEN THE OPENING. Nothing has come "
                "off /tube_hole that survived the gating, so the crossing "
                "height is the TEMPLATE's, and the template cannot tell which "
                "way the diagonal slopes. If it is the wrong way round this is "
                "the run that hits it. Check the overlay: the cell should be "
                "outlined and crossed.")
        self.MOVE_SPEED = self.TUBE_PASS_SPEED
        self._enter_tube_stage(self.TUBE_PASS)
        self._set_target(self.tube_pass_exit[0], self.tube_pass_exit[1],
                         self.tube_altitude)
        self.get_logger().warning(
            "TUBE_PASS: committed, through the gap. The camera is no longer "
            "steering.")

    def _handle_tube_pass(self):
        self.yaw_remaining = wrap_pi(self.gap_heading - self.yaw_setpoint)
        now = time.monotonic()
        if not self.hold_xy:
            past = self._tube_past_plane()
            if past < -0.35 and self.tube_push_since is None:
                self._abandon("flow lost before the gap; landing in front of it")
                return
            if self.tube_push_since is None:
                remaining = max(0.0, self.TUBE_PASS_EXIT - past)
                self.tube_push_seconds = remaining / max(self.TUBE_PASS_SPEED, 1e-3)
                self.tube_push_since = now
                self.get_logger().error(
                    f"TUBE_PASS: flow lost in the gap. Pushing on open-loop "
                    f"{remaining:.2f} m before landing.")
            elif now - self.tube_push_since >= self.tube_push_seconds:
                self._hold_and_wait(self.TUBE_SHIFT,
                                    "flow lost in the gap; pushed through open-loop")
            return
        if self.tube_push_since is not None:
            self.get_logger().warning("TUBE_PASS: flow is back; resuming.")
            self.tube_push_since = None

        if self._tube_arrived(self.tube_pass_exit):
            if not self._range_ok_or_hold():
                return
            self.MOVE_SPEED = self.TUBE_SHIFT_SPEED
            if self.TUBE_MERGE_SHIFT:
                # One diagonal leg from just past the plane to clear of the
                # back upright, rather than a sideways step and then a
                # straight run. The aircraft stays square to the gap while it
                # is BETWEEN the uprights -- under 10 cm a side, no room to be
                # going sideways -- and eases left the moment it is out.
                self._enter_tube_stage(self.TUBE_EXIT)
                self._set_target(self.tube_final_point[0],
                                 self.tube_final_point[1], self.tube_altitude)
                self.get_logger().warning(
                    f"TUBE_EXIT: through. One diagonal leg {self.tube_shift:+.2f} m "
                    f"({'left' if self.tube_shift >= 0 else 'right'}) and "
                    f"{self.tube_exit_distance():.2f} m on, round the back upright.")
                return
            self._enter_tube_stage(self.TUBE_SHIFT)
            self._set_target(self.tube_shift_point[0], self.tube_shift_point[1],
                             self.tube_altitude)
            self.get_logger().warning(
                f"TUBE_SHIFT: through. {self.tube_shift:+.2f} m "
                f"({'left' if self.tube_shift >= 0 else 'right'}) to clear the "
                "upright behind the plane.")
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._abandon("the tube pass timed out")
            return
        self.get_logger().info(
            f"TUBE_PASS: {self._tube_past_plane():+.2f} m past the plane.",
            throttle_duration_sec=0.5)

    def _handle_tube_shift(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._hold_and_wait(self.TUBE_SHIFT, "flow lost during the tube shift")
            return
        if self._tube_arrived(self.tube_shift_point):
            self.MOVE_SPEED = self.APPROACH_SPEED
            self._enter_tube_stage(self.TUBE_EXIT)
            self._set_target(self.tube_final_point[0], self.tube_final_point[1],
                             self.tube_altitude)
            self.get_logger().warning(
                f"TUBE_EXIT: {self.tube_exit_distance():.2f} m on, clear past "
                "the back upright.")
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._abandon("the tube shift timed out")

    def _handle_tube_exit(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._hold_and_wait(self.TUBE_EXIT, "flow lost on the way out")
            return
        # Past the back upright with the sideways step flown is what this
        # leg is FOR, and it is a weaker test than "within 10 cm of a point":
        # the aircraft cannot fail to satisfy it by drifting, which is how it
        # used to end up timing out and landing next to the upright.
        past = self._tube_past_plane()
        _, cross = self._tube_frame(self.tube_final_point)
        if (past >= self.tube_exit_distance() - self.TUBE_ARRIVE_TOLERANCE
                and cross is not None and abs(cross) <= self.TUBE_CROSS_CLEAR):
            self._finish_tubes("course complete")
            return
        if self._tube_arrived(self.tube_final_point):
            self._finish_tubes("course complete")
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            # Being past the back upright is the whole point of the exit. If
            # the aircraft is clear of it, finish rather than land alongside
            # it -- a plain timeout here used to leave it hovering at the
            # upright, which is exactly where it must not stop.
            past = self._tube_past_plane()
            if past > self.tube_back_along + self.TUBE_CLEARANCE:
                self.get_logger().warning(
                    f"TUBE_EXIT: did not settle, but {past:.2f} m past the "
                    f"plane is clear of the back upright at "
                    f"{self.tube_back_along:.2f} m. Calling it done.")
                self._finish_tubes("course complete (exit not settled)")
                return
            self._abandon("the tube exit timed out short of the back upright")

    def _finish_tubes(self, reason):
        self.tubes_done = True
        self.set_detection('tube', False)
        if self.WINDOW_AFTER_TUBES and self.window_pass < 2:
            self._begin_window2_rise()
            return
        self._after_the_course(reason)

    def _after_the_course(self, reason):
        """Everything that follows the last obstacle: the pad, or a landing."""
        if not self.PAD:
            if not self._range_ok_or_hold():
                return
            self.outcome = (f"COURSE COMPLETE: window, red bar, both blue bars, "
                            f"tube gap at {self.tube_altitude:.2f} m"
                            + (", second window" if self.window_pass >= 2 else ""))
            self._begin_landing(reason)
            return
        self._begin_pad_offset()

    # ------------------------------------------------- the SECOND window

    def _begin_window2_rise(self):
        """Climb back to the search altitude, straight up, where we stand.

        The tubes end at the gap altitude -- under the cross tube, which is
        the lowest thing on the course -- and the window mission started from
        one that cannot be measured from: ALIGN flies to a standoff point at
        the WINDOW's height, so from down here the run-up to it is a long
        diagonal climb through whatever the aircraft has just squeezed under.
        Getting the height back first, on the spot, makes the second pass the
        same flight the first one was: hold at the search altitude, look,
        line up, go through.

        Straight up and nowhere else: the aircraft is a metre or so past the
        back upright and the only clear direction from here is up.
        """
        lp = self.local_position
        if lp is not None:
            x, y = lp.x, lp.y
        elif self.move_target_x is not None:
            x, y = self.move_target_x, self.move_target_y
        else:
            # No position at all: there is nothing to climb on and nothing to
            # search from. Put it down where it is instead of pushing a
            # setpoint at an estimator that has nothing to say.
            self._begin_landing("no position estimate to start the second "
                                "window from")
            return
        self.CLIMB_SPEED = self.COURSE_CLIMB_SPEED
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(x, y, self.WINDOW2_ALTITUDE)
        self._enter_course_stage(self.WINDOW2_RISE)
        self.get_logger().warning(
            f"WINDOW2_RISE: tubes done. Climbing straight up to "
            f"{self.WINDOW2_ALTITUDE:.2f} m, holding position, before looking "
            "for the second window.")

    def _handle_window2_rise(self):
        self._handle_vertical(self._begin_window2_hold,
                              "the climb back to the window altitude")

    def _begin_window2_hold(self):
        self._enter_course_stage(self.WINDOW2_HOLD)
        self.get_logger().warning(
            f"WINDOW2_HOLD: at {self.WINDOW2_ALTITUDE:.2f} m. Standing still "
            f"for {self.WINDOW2_HOLD_SECONDS:.1f} s, then the search.")

    def _handle_window2_hold(self):
        """Hover, stationary, so the search starts from a settled camera."""
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        self._aim_yaw_at(self._course_heading())

        lost = self._flow_lost_for()
        if lost > self.COURSE_FLOW_TIMEOUT:
            self._hold_and_wait(self.WINDOW2_HOLD,
                                "optical flow lost before the second window")
            return

        remaining = self.WINDOW2_HOLD_SECONDS - self._in_stage_for()
        if remaining <= 0.0:
            self._begin_second_window()
            return
        self.get_logger().info(
            f"WINDOW2_HOLD: {remaining:.1f} s to the search.",
            throttle_duration_sec=0.5)

    def _window2_search_stalled(self):
        """True once the second window has been looked for long enough."""
        return (self.window_pass >= 2 and not self.window2_skipped
                and self.WINDOW2_SKIP
                and self.current_stage in (self.SCAN, self.LOCK)
                and self._in_stage_for() > self.WINDOW2_SEARCH_TIMEOUT
                and not self.window_is_confirmed())

    def _handle_scan(self):
        """The inherited sweep, with a deadline on the SECOND window.

        The first window has nothing to fall back on -- the whole course is
        laid out from it -- so it sweeps until the flight clock says
        otherwise. The second one does: everything behind it is already
        flown, so a window that never appears is skipped rather than sat in
        front of until the battery decides the matter.
        """
        if self._window2_search_stalled():
            self._begin_window2_skip(
                f"no window found in {self.WINDOW2_SEARCH_TIMEOUT:.0f} s of "
                "sweeping")
            return
        super()._handle_scan()

    def _abandon(self, reason):
        """Give up -- but on the SECOND window, skip it instead of landing.

        Every way the window mission can fail (the pose never converging, the
        approach not settling, the vision going away during ALIGN) ends in
        _abandon, and on the second pass none of them is worth a landing: the
        course behind is flown, and there is a way past the window that needs
        no camera at all.
        """
        if (self.window_pass >= 2 and self.WINDOW2_SKIP
                and not self.window2_skipped
                and self.current_stage not in self.COURSE_STAGES):
            self._begin_window2_skip(reason)
            return
        super()._abandon(reason)

    # ---- skipping it: over the wall, across, and back down ----------------

    def _window2_skip_distance(self):
        """How far forward the skip flies: the traverse it is replacing."""
        return self.STANDOFF_DISTANCE + self.EXIT_DISTANCE

    def _begin_window2_skip(self, why):
        """Fly OVER the second window's wall instead of through the window.

        The aircraft is at the search altitude in front of a window it cannot
        see, with the whole course behind it already flown. Landing here
        throws that away. The red bar's crossing altitude is a height this
        aircraft has already held once on this flight, above everything the
        course stands up, so the skip climbs to it, crosses the distance the
        traverse would have covered, and comes back down to the search
        altitude on the far side.

        It is the failsafe, not the plan: the window is tried first, for
        window2_search_timeout, and only a window that never appears gets
        this. And it assumes nothing stands ABOVE the red bar's altitude on
        the course line -- which is true of this course, and is the one thing
        to check before enabling it on another.
        """
        self.window2_skipped = True
        self.set_detection('window', False)
        self.moving = False
        self.yaw_remaining = 0.0
        lp = self.local_position
        if lp is None:
            super()._abandon(f"{why}, and there is no position to skip on")
            return
        self.CLIMB_SPEED = self.COURSE_CLIMB_SPEED
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(lp.x, lp.y, self.red_altitude)
        self._enter_course_stage(self.WINDOW2_SKIP_RISE)
        self.get_logger().error(
            f"WINDOW2 SKIPPED: {why}. NOT landing. Climbing to the red bar's "
            f"{self.red_altitude:.2f} m -- above everything this course "
            f"stands up -- then {self._window2_skip_distance():.2f} m forward "
            f"(the traverse this replaces) and back down to "
            f"{self.WINDOW2_ALTITUDE:.2f} m.")

    def _handle_window2_skip_rise(self):
        self._handle_vertical(self._begin_window2_skip_cross,
                              "the climb over the second window")

    def _begin_window2_skip_cross(self):
        distance = self._window2_skip_distance()
        x, y = self._ahead(distance)
        self.MOVE_SPEED = self.BAR_CROSS_SPEED
        self._set_target(x, y, self.red_altitude)
        self._enter_course_stage(self.WINDOW2_SKIP_CROSS)
        self.get_logger().warning(
            f"WINDOW2_SKIP_CROSS: {distance:.2f} m forward at "
            f"{self.red_altitude:.2f} m, over the window rather than through "
            "it.")

    def _handle_window2_skip_cross(self):
        self._handle_crossing(self._begin_window2_skip_drop,
                              "the skip across the second window")

    def _begin_window2_skip_drop(self):
        x, y = self.move_target_x, self.move_target_y
        self.course_saved_land_speed = self.LAND_SPEED
        self.LAND_SPEED = self.COURSE_DESCENT_SPEED
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(x, y, self.WINDOW2_ALTITUDE)
        self._enter_course_stage(self.WINDOW2_SKIP_DROP)
        self.get_logger().warning(
            f"WINDOW2_SKIP_DROP: back down to {self.WINDOW2_ALTITUDE:.2f} m, "
            "then the landing the course finishes with.")

    def _handle_window2_skip_drop(self):
        self._handle_vertical(self._finish_window2_skip,
                              "the descent after the second window")

    def _finish_window2_skip(self):
        self._restore_land_speed()
        self._after_the_course("second window skipped -- never detected")

    def _begin_second_window(self):
        """Fly the window mission again, from wherever the tubes ended.

        The whole window state machine is reused as-is, so this only has to
        put the node back into the state it was in before the first window:
        the detector on, the estimator empty -- every sample in it is of the
        FIRST window, metres behind us, and feeding those to the second
        traverse would fly the aircraft at a window it has already been
        through -- and the committed traverse, the pose health and the
        RECENTRE bookkeeping all cleared. Then SCAN, exactly as after the
        take-off hold.
        """
        self.window_pass = 2
        if self.WINDOW2_EXIT_DISTANCE > 0.0:
            self.EXIT_DISTANCE = self.WINDOW2_EXIT_DISTANCE

        self.estimator.samples.clear()
        self.pose_ok_since = None
        self.pose_lost_since = None
        self.last_good_est = None
        self.last_good_est_time = 0.0
        self.last_detection = None
        self.truncated_frames = 0
        self.recentre_untruncated_since = None
        self.recentre_backoffs = 0
        self.recentre_backoff_target = None
        self.recentre_attempt_since = None
        self.traverse_entry = None
        self.traverse_exit = None
        self.traverse_heading = None
        self.traverse_window = None
        self.blind_traverse_since = None
        self.align_in_band_since = None
        self.window_blind_capped = False
        self.BLIND_TRAVERSE_SECONDS = self._blind_seconds_param

        self.MOVE_SPEED = self.APPROACH_SPEED
        self.set_detection('window', True)
        self.get_logger().warning(
            "SECOND WINDOW: tubes done. Looking for the next window and "
            f"flying the whole traverse again, ending {self.EXIT_DISTANCE:.2f} m "
            "past it, and then the landing the course would have finished "
            "with.")
        self._begin_scan()

    # ----------------------------------------------------------- THE PAD

    def aruco_detected_callback(self, msg):
        self.aruco_flag = bool(msg.data)

    def aruco_point_callback(self, msg):
        """The marker in the CAMERA body frame -> (forward, right) in metres.

        aruco_pose publishes +x right in the image, +y up in the image and +z
        opposite to where the camera looks. With the camera pointing down and
        its image-up towards the nose, that is forward = y, right = x, and
        height = -z (see its docstring; precision_land.py maps it the same
        way). Only forward and right are taken: the height from a marker is
        scaled by whatever error the quoted field of view carries, and the
        rangefinder knows better.
        """
        self.aruco_seen += 1
        self.aruco_point = (float(msg.point.y), float(msg.point.x))
        self.aruco_point_time = time.monotonic()

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

    # ------------------------------------------------- off the takeoff pad

    def _handle_hold(self):
        """The post-takeoff hold, with one lateral step bolted to its end.

        The hook is HERE and not in _begin_scan because the inherited hold
        has two exits, not one: if the detector is already confirming a
        window when the hold runs out it locks on immediately and the sweep
        is never begun. From the takeoff pad that shortcut is exactly the
        wrong thing to take -- whatever the camera thinks it can see from the
        pad, the aircraft is not on the window's axis yet -- so the step goes
        in front of BOTH exits, and the decision between them is deferred to
        _end_of_hold once the aircraft has moved.
        """
        if self.start_offset_done or abs(self.START_OFFSET_RIGHT) < 1e-3:
            super()._handle_hold()
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        remaining = self.HOLD_SECONDS - self._in_stage_for()
        if remaining > 0.0:
            self.get_logger().info(
                f"Holding, {remaining:.1f} s to the step off the pad... "
                f"({self.window_summary()})",
                throttle_duration_sec=1.0)
            self.log_flight_state()
            return
        if self.local_position is None or not self.hold_xy:
            # No position hold means no lateral step worth flying: a carrot
            # walked on velocity hold alone goes an unknown distance. Give
            # the step up rather than guess, and let the inherited hold take
            # whichever of its exits it wants.
            self.start_offset_done = True
            self.get_logger().error(
                "START_OFFSET: no xy position hold at the end of the hold; "
                "skipping the step off the pad and searching from here.")
            super()._handle_hold()
            return
        self._begin_start_offset()

    def _end_of_hold(self):
        """The inherited end-of-hold decision, taken after the step."""
        if self.window_is_confirmed():
            self._lock_on_window("in sight after the step off the pad")
        else:
            self._begin_scan()

    def _begin_start_offset(self):
        """One step sideways off the pad, holding the arming heading."""
        self.start_offset_done = True
        # Right of the ARMING heading, not of the current one: this runs
        # before any window is flown at, so home_yaw is the only direction
        # the flight has agreed on, and it is the one the operator aimed.
        h = self.home_yaw
        right = np.array([-math.sin(h), math.cos(h)])
        lp = self.local_position
        target = np.array([lp.x, lp.y]) + right * self.START_OFFSET_RIGHT
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_stage(self.START_OFFSET)
        self._set_target(float(target[0]), float(target[1]),
                         self.commanded_altitude)
        self.get_logger().warning(
            f"START_OFFSET: {abs(self.START_OFFSET_RIGHT):.2f} m "
            f"{'RIGHT' if self.START_OFFSET_RIGHT >= 0.0 else 'LEFT'} off the "
            f"takeoff pad at {self.MOVE_SPEED:.2f} m/s, holding "
            f"{math.degrees(h):+.0f} deg, then the window search.")

    def _handle_start_offset(self):
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        self.log_flight_state()
        if not self.hold_xy:
            self._hold_and_wait(self.START_OFFSET, "flow lost stepping off the pad")
            return
        self._aim_yaw_at(self.home_yaw)
        lp = self.local_position
        left = (None if lp is None or self.move_target_x is None else
                math.hypot(self.move_target_x - lp.x, self.move_target_y - lp.y))
        timed_out = self._in_stage_for() > self.START_OFFSET_TIMEOUT
        if (left is not None and left <= self.COURSE_XY_TOLERANCE) or timed_out:
            self.moving = False
            self.get_logger().warning(
                f"START_OFFSET: off the pad"
                f"{' (timed out)' if timed_out else ''}"
                f"{'' if left is None else f', {left:.2f} m short'}"
                f". Starting the window search. ({self.window_summary()})")
            self._end_of_hold()
            return
        self.get_logger().info(
            f"START_OFFSET: {0.0 if left is None else left:.2f} m to go.",
            throttle_duration_sec=1.0)

    def _begin_pad_offset(self):
        """One step to the RIGHT, off the line the obstacles stand on."""
        h = self._course_heading()
        right = np.array([-math.sin(h), math.cos(h)])
        lp = self.local_position
        target = np.array([lp.x, lp.y]) + right * self.PAD_RIGHT
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._enter_tube_stage(self.PAD_OFFSET)
        self._set_target(float(target[0]), float(target[1]), self.tube_altitude)
        self.get_logger().warning(
            f"PAD_OFFSET: tubes done. {self.PAD_RIGHT:.2f} m RIGHT, clear of "
            "the line the obstacles stand on, then forward looking for the "
            "marker.")

    def _handle_pad_offset(self):
        self._aim_yaw_at(self.gap_heading if self.gap_heading is not None
                         else self.traverse_heading)
        if not self.hold_xy:
            self._hold_and_wait(self.PAD_OFFSET, "flow lost stepping off the line")
            return
        along, cross = self._target_errors()
        if (along is not None and abs(along) <= self.COURSE_XY_TOLERANCE
                and abs(cross) <= self.COURSE_XY_TOLERANCE) or \
                self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
            self._begin_pad_search()
            return
        self.get_logger().info(
            f"PAD_OFFSET: {0.0 if cross is None else cross:+.2f} m across to go.",
            throttle_duration_sec=1.0)

    def _begin_pad_search(self):
        lp = self.local_position
        self.pad_search_start = np.array([lp.x, lp.y])
        x, y = self._ahead(self.PAD_SEARCH_DISTANCE)
        self.MOVE_SPEED = self.PAD_SEARCH_SPEED
        self._enter_tube_stage(self.PAD_SEARCH)
        self._set_target(x, y, self.tube_altitude)
        self.get_logger().warning(
            f"PAD_SEARCH: forward at {self.PAD_SEARCH_SPEED:.2f} m/s, up to "
            f"{self.PAD_SEARCH_DISTANCE:.2f} m, until the marker is under us.")

    def _handle_pad_search(self):
        if not self.hold_xy:
            self._hold_and_wait(self.PAD_SEARCH, "flow lost looking for the marker")
            return
        if self.aruco_flag and self._aruco_fresh():
            self._begin_pad_centre()
            return
        lp = self.local_position
        gone = float(np.linalg.norm(np.array([lp.x, lp.y]) - self.pad_search_start))
        timed_out = self._in_stage_for() > self.PAD_STAGE_TIMEOUT
        if gone >= self.PAD_SEARCH_DISTANCE - self.COURSE_XY_TOLERANCE or timed_out:
            # Not a failure. The course is flown; there is simply no marker
            # here to land ON. Landing where we are is the right ending, and
            # calling it an abandonment buries that in an error.
            self.outcome = ("COURSE COMPLETE: window, red bar, both blue bars, "
                            f"tubes. No landing marker found in {gone:.2f} m "
                            f"of looking ({self.aruco_seen} poses seen).")
            self.get_logger().warning(
                f"PAD_SEARCH: no marker in {gone:.2f} m"
                f"{' (timed out)' if timed_out else ''}. The course is flown; "
                "landing here.")
            self._begin_landing("course complete, no marker to land on")
            return
        self.get_logger().info(
            f"PAD_SEARCH: {gone:.2f}/{self.PAD_SEARCH_DISTANCE:.2f} m, "
            f"marker {'YES' if self.aruco_flag else 'no'}.",
            throttle_duration_sec=1.0)

    def _begin_pad_centre(self):
        self.MOVE_SPEED = self.PAD_SEARCH_SPEED
        self._enter_tube_stage(self.PAD_CENTRE)
        self.pad_settle_since = None
        self._nudge_onto_marker()
        self.get_logger().warning("PAD_CENTRE: marker found. Centring over it.")

    def _nudge_onto_marker(self):
        """Move the hold point onto the marker, a fraction of the error at a time."""
        offset = self._aruco_ned()
        if offset is None:
            return None
        lp = self.local_position
        step = offset * self.PAD_GAIN
        n = float(np.linalg.norm(step))
        if n > self.PAD_MAX_NUDGE:
            step = step / n * self.PAD_MAX_NUDGE
        target = np.array([lp.x, lp.y]) + step
        self._set_target(float(target[0]), float(target[1]),
                         self.commanded_altitude)
        return float(np.linalg.norm(offset))

    def _handle_pad_centre(self):
        if not self.hold_xy:
            self._hold_and_wait(self.PAD_CENTRE, "flow lost over the marker")
            return
        error = self._nudge_onto_marker()
        now = time.monotonic()
        if error is None:
            self.pad_settle_since = None
            if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
                self._abandon("lost the marker while centring")
            return
        if error <= self.PAD_CENTRE_TOLERANCE:
            if self.pad_settle_since is None:
                self.pad_settle_since = now
            elif now - self.pad_settle_since >= self.PAD_CENTRE_SECONDS:
                self._begin_pad_descend()
            return
        self.pad_settle_since = None
        if self._in_stage_for() > self.PAD_STAGE_TIMEOUT:
            self.get_logger().warning(
                f"PAD_CENTRE: still {error:.2f} m off after "
                f"{self.PAD_STAGE_TIMEOUT:.0f} s. Going down anyway.")
            self._begin_pad_descend()
            return
        self.get_logger().info(f"PAD_CENTRE: {error:.2f} m off the marker.",
                               throttle_duration_sec=1.0)

    def _begin_pad_descend(self):
        self._enter_tube_stage(self.PAD_DESCEND)
        self.pad_lost_since = None
        self.get_logger().warning(
            f"PAD_DESCEND: down at {self.PAD_DESCENT_RATE:.2f} m/s, correcting "
            f"off the marker, until {self.PAD_HANDOFF_HEIGHT:.2f} m.")

    def _handle_pad_descend(self):
        """Walk the setpoint down while the marker keeps saying where it is.

        Losing it stops the descent and holds; it does not keep going blind.
        Regaining it resumes. Below the handoff height the marker no longer
        fits in the frame, so PX4's land takes the last part.
        """
        if not self.hold_xy:
            self._hold_and_wait(self.PAD_DESCEND, "flow lost during the descent")
            return
        now = time.monotonic()
        alt = self.relative_altitude()
        error = self._nudge_onto_marker()

        if error is None:
            if self.pad_lost_since is None:
                self.pad_lost_since = now
                self.get_logger().error(
                    "PAD_DESCEND: marker lost. Holding altitude until it is back.")
            if now - self.pad_lost_since > self.PAD_STAGE_TIMEOUT:
                self._abandon("the marker never came back; landing from here")
            return
        if self.pad_lost_since is not None:
            self.get_logger().warning("PAD_DESCEND: marker back; resuming.")
            self.pad_lost_since = None

        if alt is not None and alt <= self.PAD_HANDOFF_HEIGHT:
            self.outcome = ("COURSE COMPLETE: window, red bar, both blue bars, "
                            f"tubes, landing on the marker {error:.2f} m off centre")
            self._begin_landing("over the marker, handing the last "
                                f"{self.PAD_HANDOFF_HEIGHT:.2f} m to PX4")
            return
        self.commanded_altitude = max(
            self.PAD_HANDOFF_HEIGHT,
            self.commanded_altitude - self.PAD_DESCENT_RATE * self.TIMER_PERIOD)
        self.target_z = self.home_z - self.commanded_altitude
        self.get_logger().info(
            f"PAD_DESCEND: alt {'n/a' if alt is None else f'{alt:.2f}'} -> "
            f"{self.commanded_altitude:.2f} m, {error:.2f} m off the marker.",
            throttle_duration_sec=1.0)

    # ------------------------------------------------------ plumbing

    def _still_flyable(self):
        """The base checks, except that a lidar dropout hovers instead of lands.

        OffboardSequence lands the moment rangefinder fusion stops, and it is
        right to in general: the height estimate then free-runs on the IMU.
        But on this course the fusion stops for a KNOWN and temporary reason --
        the lidar sweeping over the red bar or the cross tube -- and landing
        from between two obstacles is worse than holding still for a few
        seconds while EKF2 sorts itself out. So here it goes to COURSE_HOLD,
        which hovers, waits, and resumes; and if it never comes back,
        course_hold_timeout lands it after all.
        """
        if self.current_stage not in self.COURSE_STAGES:
            return super()._still_flyable()

        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().warning("Vehicle disarmed by PX4. Stopping.")
            self._stand_down()
            return False
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.get_logger().error(
                "Offboard lost; PX4 has control now "
                f"(nav_state {nav_state_name(self.nav_state)}). Standing down. "
                f"PX4 failsafe: {self.failsafe_summary()}.")
            self._stand_down()
            return False

        lp = self.local_position
        if lp is None or not lp.z_valid:
            # Not the same thing: no height estimate at all is unrecoverable.
            self._begin_landing("height estimate went invalid")
            return False

        if self.rangefinder_is_healthy():
            if self.course_pressed_on:
                self.get_logger().warning(
                    "The rangefinder is being fused again after the course "
                    "pressed on without it. Back to normal.")
                self.course_pressed_on = False
        elif self.course_pressed_on and self._height_without_rangefinder():
            # Already decided, once, that this outage is survivable. Holding
            # again every tick would be a loop: hold, time out, press on,
            # hold.
            self.get_logger().error(
                "Still no rangefinder fusion; flying on the baro height and "
                "the flow hold, as decided when the hold timed out.",
                throttle_duration_sec=5.0)
            return True

        if not self.rangefinder_is_healthy() and self.current_stage != self.COURSE_HOLD:
            # Mid-crossing with a good flow hold: fly it out, then hover clear.
            if self.current_stage in self.FINISH_FIRST_STAGES and self.hold_xy:
                self.get_logger().error(
                    "EKF2 stopped fusing the rangefinder mid-crossing. Flow hold "
                    "is good, so finishing the crossing before hovering -- "
                    "stopping here would wait over the obstacle that broke it.",
                    throttle_duration_sec=2.0)
                return True
            self._hold_and_wait(self.current_stage,
                                "EKF2 stopped fusing the rangefinder")
            return False
        return True

    def _range_ok_or_hold(self):
        """True if the height estimate is anchored; otherwise hover and wait.

        Called where a stage is about to hand over to a CLIMB, a DESCENT or a
        landing. Those are the moves that need a trustworthy height, so they
        wait for one; level flight does not and has already been let through.
        """
        if self.rangefinder_is_healthy():
            return True
        if self.course_pressed_on and self._height_without_rangefinder():
            self.get_logger().error(
                "Handing over to a vertical move with no rangefinder fusion, "
                "on the baro height. Already decided (course_hold_press_on); "
                "not holding again.", throttle_duration_sec=5.0)
            return True
        self._hold_and_wait(self.current_stage,
                            "EKF2 stopped fusing the rangefinder")
        return False

    def _hold_and_wait(self, resume_stage, reason):
        """Stop where we are, hover, and remember what to go back to."""
        if self.current_stage == self.COURSE_HOLD:
            return
        self.course_resume_stage = resume_stage
        self.course_hold_reason = reason
        self.course_hold_since = time.monotonic()
        self.moving = False
        lp = self.local_position
        if lp is not None and self.hold_xy:
            self.hold_x, self.hold_y = lp.x, lp.y
        self._enter_stage(self.COURSE_HOLD)
        self.get_logger().error(
            f"COURSE_HOLD: {reason} during {resume_stage}. Hovering here and "
            f"waiting up to {self.COURSE_HOLD_TIMEOUT:.0f} s for it to come "
            "back, then carrying on. NOT landing.")

    def _resume_from_hold(self, why):
        """Leave COURSE_HOLD and pick the interrupted stage back up."""
        resume = self.course_resume_stage or self.RED_RISE
        self.course_hold_since = None
        self.course_rng_ok_since = None
        self.moving = resume not in (self.TUBE_SEARCH, self.TUBE_LOCK,
                                     self.TUBE_SCAN, self.WINDOW2_HOLD)
        self._enter_stage(resume)
        self.get_logger().warning(f"COURSE_HOLD: {why}. Resuming {resume}.")

    def _height_without_rangefinder(self):
        """True if the EKF still has a height and a position hold without it.

        With EKF2_BARO_CTRL on, losing the range aid costs the HAGL and not
        the height: z_valid stays true on the barometer. That is enough to
        fly a bar whose height is known in advance; it is NOT enough to land
        on, which is why this only ever lets the course continue.
        """
        lp = self.local_position
        return (lp is not None and lp.z_valid and lp.v_z_valid
                and self.hold_xy and self.relative_altitude() is not None)

    def _handle_course_hold(self):
        """Hover, dead still, until the estimate is back AND has stayed back.

        This is the stage that catches the rangefinder outage over the red bar
        -- the lidar steps a bar's height in one frame, EKF2 declares the
        range kinematically inconsistent and stops fusing it, and on the
        hardware logs it took about ten seconds to come back. Through all of
        it the aircraft holds this exact point: no forward drift, no descent,
        the height carried by the barometer while the range is out (which
        needs EKF2_BARO_CTRL enabled -- see config/px4_sitl_imav.rcS).

        Nothing resumes on the first healthy frame. The flag flickers as the
        fusion re-establishes, so it has to stay healthy for
        course_hold_confirm_seconds before the next obstacle is attempted.
        """
        self._try_latch_xy_hold()
        self.log_flight_state()
        now = time.monotonic()
        waited = now - (self.course_hold_since or now)

        healthy = self.rangefinder_is_healthy() and self.hold_xy
        if not healthy:
            self.course_rng_ok_since = None
        elif self.course_rng_ok_since is None:
            self.course_rng_ok_since = now
            self.get_logger().warning(
                f"COURSE_HOLD: the rangefinder is back after {waited:.1f} s. "
                f"Holding {self.COURSE_HOLD_CONFIRM_SECONDS:.1f} s more to "
                "confirm the fusion is steady before moving.")
        elif now - self.course_rng_ok_since >= self.COURSE_HOLD_CONFIRM_SECONDS:
            self._resume_from_hold(
                f"{self.course_hold_reason} cleared after {waited:.1f} s and "
                f"held for {self.COURSE_HOLD_CONFIRM_SECONDS:.1f} s")
            return

        if waited > self.COURSE_HOLD_TIMEOUT:
            if self.COURSE_HOLD_PRESS_ON and self._height_without_rangefinder():
                self._resume_from_hold(
                    f"{self.course_hold_reason} did NOT clear in "
                    f"{self.COURSE_HOLD_TIMEOUT:.0f} s, but the EKF still has "
                    "a height (baro) and the flow still holds position, so "
                    "the course goes on rather than landing here. The "
                    "obstacle ahead is flown on known geometry; watch the "
                    "height. Set course_hold_press_on:=false to land instead")
                self.course_pressed_on = True
                return
            self._abandon(
                f"{self.course_hold_reason} did not clear in "
                f"{self.COURSE_HOLD_TIMEOUT:.0f} s of hovering"
                + ("" if self.COURSE_HOLD_PRESS_ON else
                   " (course_hold_press_on is false)")
                + (" and there is no usable height without it"
                   if self.COURSE_HOLD_PRESS_ON else ""))
            return

        lp = self.local_position
        self.get_logger().warning(
            f"COURSE_HOLD: waiting ({self.course_hold_reason}); "
            f"rangefinder_ok={self.rangefinder_is_healthy()} flow_hold={self.hold_xy} "
            f"lidar={'n/a' if lp is None else f'{lp.dist_bottom:.2f}'} m, "
            f"{self.COURSE_HOLD_TIMEOUT - waited:.0f} s left.",
            throttle_duration_sec=1.0)

    def flow_is_healthy(self):
        """Flow fusion alone once bars and tubes are in play.

        The base class also demands EKF2's rangefinder consistency flag. That
        flag drops when the lidar sweeps over the red bar or the cross tube --
        a two-metre step in dist_bottom -- and it only re-earns itself above
        0.5 m/s of vertical speed, so it never returns in a hover. Requiring
        it is what made the aircraft give up and land just after the red bar.
        """
        if self.current_stage not in self.FLOW_ONLY_STAGES:
            return super().flow_is_healthy()
        lp = self.local_position
        f = self.estimator_flags
        if f is None:
            return super().flow_is_healthy()
        return (lp is not None and lp.xy_valid and lp.v_xy_valid
                and lp.dist_bottom > self.FLOW_MIN_AGL
                and f.cs_opt_flow and not f.cs_inertial_dead_reckoning)

    def _on_heading_reset(self, delta):
        super()._on_heading_reset(delta)
        lp = self.local_position
        if lp is None:
            return
        pivot = np.array([lp.x, lp.y])
        c, sn = math.cos(delta), math.sin(delta)

        def turn(p):
            d = np.asarray(p) - pivot
            return pivot + np.array([c * d[0] - sn * d[1], sn * d[0] + c * d[1]])

        def spin(v):
            return np.array([c * v[0] - sn * v[1], sn * v[0] + c * v[1]])

        self.tube_estimator.rotate((lp.x, lp.y), delta)
        for name in ('gap_point', 'tube_entry', 'tube_pass_exit',
                     'tube_shift_point', 'tube_final_point'):
            if getattr(self, name, None) is not None:
                setattr(self, name, turn(getattr(self, name)))
        for name in ('gap_normal', 'gap_left'):
            if getattr(self, name, None) is not None:
                setattr(self, name, spin(getattr(self, name)))
        if self.gap_heading is not None:
            self.gap_heading = wrap_pi(self.gap_heading + delta)

    def _clock_stages(self):
        # The crossings are excluded for the same reason the window traverse
        # is: a stopwatch must not start a descent over or under a bar. The
        # vertical stages are at midpoints, where a descent passes nothing.
        return super()._clock_stages() + (self.RED_RISE, self.BLUE_DROP,
                                          self.COURSE_HOLD)

    def timer_callback(self):
        if self.current_stage in self.TUBE_STAGES:
            self._update_tube_solution()
            self.publish_tube_gap()

        if self.current_stage not in self.COURSE_STAGES:
            super().timer_callback()
            return

        if self._check_flight_clock():
            return

        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()

        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self._begin_landing("operator abort")
            return

        if self.current_stage in self.TUBE_STAGES:
            if not self._still_flyable():
                return
            self._try_latch_xy_hold()
            self.log_flight_state()

        {
            self.START_OFFSET: self._handle_start_offset,
            self.RED_RISE: self._handle_red_rise,
            self.RED_CROSS: self._handle_red_cross,
            self.BLUE_DROP: self._handle_blue_drop,
            self.BLUE_CROSS: self._handle_blue_cross,
            self.TUBE_CLIMB: self._handle_tube_climb,
            self.TUBE_APPROACH: self._handle_tube_approach,
            self.TUBE_SCAN: self._handle_tube_scan,
            self.TUBE_SEARCH: self._handle_tube_search,
            self.TUBE_LOCK: self._handle_tube_lock,
            self.TUBE_ALIGN: self._handle_tube_align,
            self.TUBE_PASS: self._handle_tube_pass,
            self.TUBE_SHIFT: self._handle_tube_shift,
            self.TUBE_EXIT: self._handle_tube_exit,
            self.WINDOW2_RISE: self._handle_window2_rise,
            self.WINDOW2_HOLD: self._handle_window2_hold,
            self.WINDOW2_SKIP_RISE: self._handle_window2_skip_rise,
            self.WINDOW2_SKIP_CROSS: self._handle_window2_skip_cross,
            self.WINDOW2_SKIP_DROP: self._handle_window2_skip_drop,
            self.PAD_OFFSET: self._handle_pad_offset,
            self.PAD_SEARCH: self._handle_pad_search,
            self.PAD_CENTRE: self._handle_pad_centre,
            self.PAD_DESCEND: self._handle_pad_descend,
            self.COURSE_HOLD: self._handle_course_hold,
        }[self.current_stage]()

    def publish_position_setpoint(self):
        """The inherited setpoint, except for the open-loop push over red."""
        bar_push = (self.current_stage in (self.RED_CROSS, self.BLUE_CROSS)
                    and self.bar_push_since is not None)
        tube_push = (self.current_stage == self.TUBE_PASS
                     and self.tube_push_since is not None)
        if not ((bar_push or tube_push) and not self.hold_xy
                and self.home_z is not None):
            super().publish_position_setpoint()
            return

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._step_setpoint_ramp()
        self._step_yaw_ramp()
        if tube_push:
            h, speed = self.gap_heading, self.TUBE_PASS_SPEED
        else:
            h, speed = self._course_heading(), self.BAR_CROSS_SPEED
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [speed * math.cos(h), speed * math.sin(h), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        if self.current_stage not in self.COURSE_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        along, _ = self._target_errors()
        if self.current_stage == self.COURSE_HOLD:
            waited = time.monotonic() - (self.course_hold_since or time.monotonic())
            detail = f"wait{self.COURSE_HOLD_TIMEOUT - waited:.0f}s"
        elif self.current_stage in self.TUBE_STAGES:
            detail = ('gap?' if self.tube_solution is None
                      else f"gap{self._tube_past_plane():+.1f}")
        elif self.current_stage in (self.RED_RISE, self.BLUE_DROP):
            detail = f"alt{self.commanded_altitude:.2f}"
        else:
            detail = f"go{0.0 if along is None else along:.1f}"
        msg = String()
        msg.data = "|".join([
            self.current_stage,
            'ARM' if armed else 'DIS',
            f"{alt:.2f}" if alt is not None else 'nan',
            'POS' if self.hold_xy else ('FLO' if self.flow_is_healthy() else '---'),
            detail,
        ])
        self.status_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().warning(f"Course outcome: {self.outcome}.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CourseFSM()
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
