"""
The obstacle course as one flight: WINDOW -> OVER the red bar -> UNDER the
blue bar -> land. ARK Flow localisation, ZED as a camera only.

    arm -> climb -> hold -> [the window mission, unchanged] -> HANDOFF ->
    RED_RISE -> RED_CROSS -> BLUE_DROP -> BLUE_CROSS -> land

    q -> abort into a controlled descent.   k -> force-disarm.

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
"""

import math
import time

import numpy as np
import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from std_msgs.msg import Bool, Float32MultiArray, String

from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.tube_cross import TubeEstimator, solve_gap
from drone_testing.tube_detect import STRIDE
from drone_testing.window_traverse import WindowTraverse


class CourseFSM(WindowTraverse):

    RED_RISE = "RED_RISE"
    RED_CROSS = "RED_CROSS"
    BLUE_DROP = "BLUE_DROP"
    BLUE_CROSS = "BLUE_CROSS"

    TUBE_CLIMB = "TUBE_CLIMB"
    TUBE_SEARCH = "TUBE_SEARCH"
    TUBE_LOCK = "TUBE_LOCK"
    TUBE_ALIGN = "TUBE_ALIGN"
    TUBE_PASS = "TUBE_PASS"
    TUBE_SHIFT = "TUBE_SHIFT"
    TUBE_EXIT = "TUBE_EXIT"

    TUBE_STAGES = (TUBE_CLIMB, TUBE_SEARCH, TUBE_LOCK, TUBE_ALIGN, TUBE_PASS,
                   TUBE_SHIFT, TUBE_EXIT)
    COURSE_STAGES = (RED_RISE, RED_CROSS, BLUE_DROP, BLUE_CROSS) + TUBE_STAGES
    # Where horizontal hold is judged on FLOW FUSION ALONE. Crossing a bar or
    # the cross tube steps dist_bottom by a metre or two, EKF2 drops
    # cs_rng_kin_consistent, and it only re-earns that at |vz| > 0.5 m/s -- so
    # it never comes back in a hover. Requiring it here is what landed the
    # aircraft just after the red bar.
    FLOW_ONLY_STAGES = (RED_CROSS, BLUE_DROP, BLUE_CROSS) + TUBE_STAGES

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
    BLUE_TO_TUBE = 2.00         # m from the second blue bar to where the
                                # aircraft stops to look for the tube obstacle

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

    FLOW_FLOOR_MARGIN = 0.10    # m the blue altitude must clear FLOW_MIN_AGL by

    # ---- the tube obstacle (same numbers and solve as tube_cross) ----------
    TUBES = True                # false = land after the blue bars
    TUBE_SPACING = 0.50
    TUBE_RADIUS = 0.025
    CROSS_BAR_HEIGHT = 0.461
    DIAGONAL_LEFT_HEIGHT = 2.000
    DIAGONAL_RIGHT_HEIGHT = 0.922
    GAP_SIDE = 'left'
    TUBE_STANDOFF = 1.20        # m before the tube plane the pass starts
    TUBE_PASS_EXIT = 0.50       # m past it before stepping sideways
    TUBE_SHIFT_LEFT = 0.40      # m left, to clear the upright behind the plane
    TUBE_EXIT_DISTANCE = 1.20
    TUBE_CLEARANCE = 0.12
    TUBE_CROSS_TOLERANCE = 0.05     # m off the gap centreline: under 10 cm a side
    TUBE_ALONG_TOLERANCE = 0.15
    TUBE_YAW_TOLERANCE = math.radians(5.0)
    TUBE_ALT_TOLERANCE = 0.06
    TUBE_SETTLE_SECONDS = 1.5
    TUBE_ARRIVE_TOLERANCE = 0.10
    TUBE_SEARCH_TIMEOUT = 45.0
    TUBE_LOCK_SECONDS = 2.0
    TUBE_STAGE_TIMEOUT = 40.0
    TUBE_PASS_SPEED = 0.30
    TUBE_SHIFT_SPEED = 0.25
    TUBE_GEOMETRY_TOPIC = 'tube_geometry'
    TUBE_DETECT_TOPIC = 'tubes_detected'
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
        self.TUBES = bool(self.declare_parameter('tubes', self.TUBES).value)
        self.TUBE_SPACING = float(n('tube_spacing', self.TUBE_SPACING))
        self.TUBE_RADIUS = float(n('tube_radius', self.TUBE_RADIUS))
        self.CROSS_BAR_HEIGHT = float(n('cross_bar_height', self.CROSS_BAR_HEIGHT))
        self.DIAGONAL_LEFT_HEIGHT = float(n('diagonal_left_height', self.DIAGONAL_LEFT_HEIGHT))
        self.DIAGONAL_RIGHT_HEIGHT = float(n('diagonal_right_height', self.DIAGONAL_RIGHT_HEIGHT))
        self.gap_side = str(self.declare_parameter('gap_side', self.GAP_SIDE).value).strip().lower()
        self.TUBE_STANDOFF = float(n('tube_standoff', self.TUBE_STANDOFF))
        self.TUBE_PASS_EXIT = float(n('tube_pass_exit', self.TUBE_PASS_EXIT))
        self.TUBE_SHIFT_LEFT = float(n('tube_shift_left', self.TUBE_SHIFT_LEFT))
        self.TUBE_EXIT_DISTANCE = float(n('tube_exit_distance', self.TUBE_EXIT_DISTANCE))
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
        )
        self.tube_altitude, self.tube_floor, self.tube_roof = self._tube_solve_altitude()

        self.create_subscription(
            Float32MultiArray,
            str(self.declare_parameter('tube_geometry_topic', self.TUBE_GEOMETRY_TOPIC).value),
            self.tube_geometry_callback, 10, callback_group=self.sensor_cbg)
        self.create_subscription(
            Bool, str(self.declare_parameter('tube_detect_topic', self.TUBE_DETECT_TOPIC).value),
            self.tube_detected_callback, 10, callback_group=self.sensor_cbg)
        self.tube_gap_pub = self.create_publisher(String, 'tube_gap', 10)

        self.tubes_flag = False
        self.tube_geometry_seen = 0
        self.tube_solution = None
        self.tube_reason = 'no data yet'
        self.tube_ok_since = None
        self.gap_point = self.gap_normal = self.gap_left = None
        self.gap_heading = None
        self.tube_entry = self.tube_pass_exit = None
        self.tube_shift_point = self.tube_final_point = None
        self.tube_settle_since = None
        self.tube_push_since = None
        self.tube_push_seconds = 0.0

        self.course_saved_land_speed = None
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
               f"{self.tube_roof:.2f} m). " if self.TUBES else "Tubes disabled. ")
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
        return self.traverse_heading

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
        """Through the window: hand off to the bars instead of landing."""
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
                self._abandon(
                    f"optical flow lost for {self.COURSE_FLOW_TIMEOUT:.0f} s during "
                    f"{what}; descending straight down on the midpoint, which "
                    "is clear of both obstacles")
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
                self._abandon(
                    "flow lost over the red bar; pushed past it open-loop and "
                    "landing on the far side")
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
            self._abandon(
                "flow lost during the blue crossing; landing immediately, which "
                "is straight down and away from the bar above")
            return

        self._handle_crossing(self._finish_course, "the blue crossing")

    def _handle_crossing(self, done, what):
        along, cross = self._target_errors()
        if along is not None and along <= self.COURSE_XY_TOLERANCE:
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

    def _tube_diagonal_height(self, lateral):
        mid = 0.5 * (self.DIAGONAL_LEFT_HEIGHT + self.DIAGONAL_RIGHT_HEIGHT)
        slope = ((self.DIAGONAL_LEFT_HEIGHT - self.DIAGONAL_RIGHT_HEIGHT)
                 / (2.0 * self.TUBE_SPACING))
        return mid + slope * lateral

    def _tube_solve_altitude(self):
        """Crossing height for the gap, solved across the whole airframe."""
        centre = (0.5 if self.gap_side == 'left' else -0.5) * self.TUBE_SPACING
        edges = (centre - 0.5 * self.DRONE_WIDTH, centre + 0.5 * self.DRONE_WIDTH)
        roof_tube = min(self._tube_diagonal_height(e) for e in edges)
        floor = (self.CROSS_BAR_HEIGHT + self.TUBE_RADIUS + self.TUBE_CLEARANCE
                 + self.body_below)
        roof = roof_tube - self.TUBE_RADIUS - self.TUBE_CLEARANCE - self.body_above
        return 0.5 * (floor + roof), floor, roof

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

    def _update_tube_solution(self):
        lp = self.local_position
        if lp is None or self.home_z is None:
            self.tube_solution = None
            return
        heading = self.gap_heading if self.gap_heading is not None else self.home_yaw
        sol, reason = solve_gap(
            self.tube_estimator.clusters(time.monotonic()), (lp.x, lp.y), heading,
            self.TUBE_SPACING, self.TUBE_PLANE_BAND, self.TUBE_MATCH_TOLERANCE,
            self.TUBE_MIN_MATCHED, self.TUBE_MAX_PLANE_YAW, self.gap_side)
        if sol is not None and not (self.TUBE_MIN_GAP_WIDTH <= sol['width']
                                    <= self.TUBE_MAX_GAP_WIDTH):
            sol, reason = None, f"measured gap {sol['width']:.2f} m is implausible"
        self.tube_solution = sol
        self.tube_reason = reason
        if sol is None:
            self.tube_ok_since = None
        elif self.tube_ok_since is None:
            self.tube_ok_since = time.monotonic()

    def tube_summary(self):
        s = self.tube_solution
        if s is None:
            if self.tube_geometry_seen == 0:
                return "nothing on /tube_geometry -- is tube_detect running?"
            return (f"no gap: {self.tube_reason} "
                    f"({self.tube_estimator.accepted_total} samples accepted; "
                    f"rejections: {self.tube_estimator.rejection_summary()})")
        return (f"gap at ({s['point'][0]:+.2f}, {s['point'][1]:+.2f}), "
                f"{s['width']:.2f} m wide, {s['matched']} uprights matched")

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
        self.tube_shift_point = self.tube_pass_exit + self.gap_left * self.TUBE_SHIFT_LEFT
        self.tube_final_point = self.tube_shift_point + self.gap_normal * self.TUBE_EXIT_DISTANCE

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

    def _begin_tube_climb(self):
        """Forward to the look-from point, rising to the gap altitude."""
        x, y = self._ahead(self.BLUE_TO_TUBE)
        self.MOVE_SPEED = self.APPROACH_SPEED
        self._set_target(x, y, self.tube_altitude)
        self._enter_tube_stage(self.TUBE_CLIMB)
        self.get_logger().warning(
            f"TUBE_CLIMB: past the blue bars. {self.BLUE_TO_TUBE:.2f} m on and "
            f"up to {self.tube_altitude:.2f} m (gap band {self.tube_floor:.2f}-"
            f"{self.tube_roof:.2f} m), then look for the uprights.")

    def _handle_tube_climb(self):
        if self._flow_lost_for() > self.COURSE_FLOW_TIMEOUT:
            self._abandon("flow lost on the way to the tubes")
            return
        along, _ = self._target_errors()
        alt = self.relative_altitude()
        settled = (along is not None and abs(along) <= self.COURSE_XY_TOLERANCE
                   and alt is not None
                   and abs(alt - self.commanded_altitude) <= self.COURSE_ALT_TOLERANCE)
        if settled or self._in_stage_for() > self.COURSE_VERTICAL_TIMEOUT:
            self._enter_tube_stage(self.TUBE_SEARCH)
            self.get_logger().warning(
                "TUBE_SEARCH: holding, looking for the uprights."
                if settled else
                "TUBE_SEARCH: did not settle on the look-from point; looking "
                "from here anyway.")
            return
        self.get_logger().info(
            f"TUBE_CLIMB: {0.0 if along is None else along:+.2f} m to go, alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.commanded_altitude:.2f} m.",
            throttle_duration_sec=1.0)

    def _handle_tube_search(self):
        if self.tube_solution is not None and self.hold_xy:
            self._enter_tube_stage(self.TUBE_LOCK)
            self.get_logger().warning(f"TUBE_LOCK: {self.tube_summary()}.")
            return
        if self._in_stage_for() > self.TUBE_SEARCH_TIMEOUT:
            self._abandon(f"no tube gap found in {self.TUBE_SEARCH_TIMEOUT:.0f} s. "
                          f"{self.tube_summary()}")
            return
        self.get_logger().info(f"TUBE_SEARCH: {self.tube_summary()}",
                               throttle_duration_sec=1.0)

    def _handle_tube_lock(self):
        if self.tube_solution is None:
            if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
                self._abandon(f"lost the tube gap. {self.tube_summary()}")
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
                self._abandon("lateral estimate never recovered before the gap")
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
            self._abandon("could not settle on the gap entry in "
                          f"{self.TUBE_STAGE_TIMEOUT:.0f} s")
            return
        self.get_logger().info(
            f"TUBE_ALIGN: {0.0 if along is None else along:+.2f} along / "
            f"{0.0 if cross is None else cross:+.2f} across (tol "
            f"{self.TUBE_CROSS_TOLERANCE:.2f}), alt "
            f"{'n/a' if alt is None else f'{alt:.2f}'}/{self.tube_altitude:.2f}. "
            f"{self.tube_summary()}", throttle_duration_sec=1.0)

    def _begin_tube_pass(self):
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
                self._abandon("flow lost in the gap; pushed through open-loop")
            return
        if self.tube_push_since is not None:
            self.get_logger().warning("TUBE_PASS: flow is back; resuming.")
            self.tube_push_since = None

        if self._tube_arrived(self.tube_pass_exit):
            self.MOVE_SPEED = self.TUBE_SHIFT_SPEED
            self._enter_tube_stage(self.TUBE_SHIFT)
            self._set_target(self.tube_shift_point[0], self.tube_shift_point[1],
                             self.tube_altitude)
            self.get_logger().warning(
                f"TUBE_SHIFT: through. {self.TUBE_SHIFT_LEFT:+.2f} m left to "
                "clear the upright behind the plane.")
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
            self._abandon("flow lost during the tube shift; landing straight down")
            return
        if self._tube_arrived(self.tube_shift_point):
            self.MOVE_SPEED = self.APPROACH_SPEED
            self._enter_tube_stage(self.TUBE_EXIT)
            self._set_target(self.tube_final_point[0], self.tube_final_point[1],
                             self.tube_altitude)
            self.get_logger().warning(
                f"TUBE_EXIT: {self.TUBE_EXIT_DISTANCE:.2f} m on, past the back "
                "upright.")
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._abandon("the tube shift timed out")

    def _handle_tube_exit(self):
        self._aim_yaw_at(self.gap_heading)
        if not self.hold_xy:
            self._abandon("flow lost on the way out; landing straight down")
            return
        if self._tube_arrived(self.tube_final_point):
            self.outcome = (f"COURSE COMPLETE: window, red bar, both blue bars, "
                            f"tube gap at {self.tube_altitude:.2f} m")
            self._begin_landing("course complete")
            return
        if self._in_stage_for() > self.TUBE_STAGE_TIMEOUT:
            self._abandon("the tube exit timed out")

    # ------------------------------------------------------ plumbing

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
        return super()._clock_stages() + (self.RED_RISE, self.BLUE_DROP)

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
            self.RED_RISE: self._handle_red_rise,
            self.RED_CROSS: self._handle_red_cross,
            self.BLUE_DROP: self._handle_blue_drop,
            self.BLUE_CROSS: self._handle_blue_cross,
            self.TUBE_CLIMB: self._handle_tube_climb,
            self.TUBE_SEARCH: self._handle_tube_search,
            self.TUBE_LOCK: self._handle_tube_lock,
            self.TUBE_ALIGN: self._handle_tube_align,
            self.TUBE_PASS: self._handle_tube_pass,
            self.TUBE_SHIFT: self._handle_tube_shift,
            self.TUBE_EXIT: self._handle_tube_exit,
        }[self.current_stage]()

    def publish_position_setpoint(self):
        """The inherited setpoint, except for the open-loop push over red."""
        if not (self.current_stage == self.RED_CROSS
                and self.bar_push_since is not None
                and not self.hold_xy and self.home_z is not None):
            super().publish_position_setpoint()
            return

        nan = float('nan')
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._step_setpoint_ramp()
        self._step_yaw_ramp()
        h = self._course_heading()
        msg.position = [nan, nan, self.setpoint_z]
        msg.velocity = [self.BAR_CROSS_SPEED * math.cos(h),
                        self.BAR_CROSS_SPEED * math.sin(h), nan]
        msg.yaw = self.yaw_setpoint
        self.trajectory_setpoint_pub.publish(msg)

    def publish_status(self):
        if self.current_stage not in self.COURSE_STAGES:
            super().publish_status()
            return
        alt = self.relative_altitude()
        armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        along, _ = self._target_errors()
        if self.current_stage in self.TUBE_STAGES:
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
