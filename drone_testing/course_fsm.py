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

import rclpy
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus
from std_msgs.msg import String

from drone_testing.offboard_sequence import spin_node
from drone_testing.window_traverse import WindowTraverse


class CourseFSM(WindowTraverse):

    RED_RISE = "RED_RISE"
    RED_CROSS = "RED_CROSS"
    BLUE_DROP = "BLUE_DROP"
    BLUE_CROSS = "BLUE_CROSS"

    COURSE_STAGES = (RED_RISE, RED_CROSS, BLUE_DROP, BLUE_CROSS)

    # ---- the course layout ------------------------------------------------
    WINDOW_TO_RED = 1.00        # m, window plane to red bar
    RED_TO_BLUE = 1.00          # m, red bar to blue bar
    BLUE_EXIT = 0.80            # m past the blue bar to stop and land. The
                                # airframe is 0.26 m across, so this puts its
                                # trailing edge well clear before the descent.

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
    COURSE_RNG_DROPOUT_TIMEOUT = 6.0  # s of rangefinder dropout tolerated in
                                # RED_CROSS. The bar passing under the lidar is
                                # a ~2 m step EKF2 rightly rejects; landing on
                                # that puts the aircraft onto the bar.

    FLOW_FLOOR_MARGIN = 0.10    # m the blue altitude must clear FLOW_MIN_AGL by

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
        self.COURSE_RNG_DROPOUT_TIMEOUT = float(n(
            'course_rng_dropout_timeout', self.COURSE_RNG_DROPOUT_TIMEOUT))

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

        self.course_saved_land_speed = None
        self.course_settle_since = None
        self.course_flow_lost_since = None
        self.bar_push_since = None
        self.bar_push_seconds = 0.0
        self.window_blind_capped = False
        self.rng_was_healthy = None     # last fusion state seen, for the log
        self.rng_lost_since = None
        self.rng_lost_stage = None
        self.rng_excused = False

        self.get_logger().warning(
            f"COURSE: window, then OVER the red bar ({self.RED_BAR_HEIGHT:.2f} m, "
            f"cross at {self.red_altitude:.2f} m), then UNDER the blue bar "
            f"({self.BLUE_BAR_HEIGHT:.2f} m, cross at {self.blue_altitude:.2f} m), "
            f"then land {self.BLUE_EXIT:.2f} m past it. Gaps "
            f"{self.WINDOW_TO_RED:.2f} / {self.RED_TO_BLUE:.2f} m; every "
            "vertical move happens at a gap midpoint. Bars are flown on the "
            "known geometry along the window's traverse line. "
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

    def _set_course_target(self, x, y, altitude):
        """_set_target, plus the move start the EKF2 reset handlers rotate.

        The inherited _set_target sets move_target_x but never move_start_x,
        and both reset handlers shift the two together. A heading reset in a
        course stage (EKF2 does one when range fusion drops over the red bar)
        then hit None - None and killed the node mid-push -- PX4 kept flying
        the last forward velocity setpoint. Course stages only.
        """
        self._set_target(x, y, altitude)
        lp = self.local_position
        self.move_start_x = float(lp.x) if lp is not None else float(x)
        self.move_start_y = float(lp.y) if lp is not None else float(y)

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
        self._set_course_target(x, y, self.red_altitude)
        self._enter_course_stage(self.RED_RISE)
        self.get_logger().warning(
            f"RED_RISE: climbing straight up to {self.red_altitude:.2f} m on the "
            f"midpoint, {0.5 * self.WINDOW_TO_RED:.2f} m short of the red bar, "
            "holding position on the window centreline.")

    def _begin_red_cross(self):
        distance = 0.5 * self.WINDOW_TO_RED + 0.5 * self.RED_TO_BLUE
        x, y = self._ahead(distance)
        self.MOVE_SPEED = self.BAR_CROSS_SPEED
        self._set_course_target(x, y, self.red_altitude)
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
        self._set_course_target(x, y, self.blue_altitude)
        self._enter_course_stage(self.BLUE_DROP)
        self.get_logger().warning(
            f"BLUE_DROP: descending straight down to {self.blue_altitude:.2f} m "
            f"on the midpoint at {self.COURSE_DESCENT_SPEED:.2f} m/s, "
            f"{0.5 * self.RED_TO_BLUE:.2f} m clear of both bars.")

    def _begin_blue_cross(self):
        self._restore_land_speed()
        distance = 0.5 * self.RED_TO_BLUE + self.BLUE_EXIT
        x, y = self._ahead(distance)
        self.MOVE_SPEED = self.BAR_CROSS_SPEED
        self._set_course_target(x, y, self.blue_altitude)
        self._enter_course_stage(self.BLUE_CROSS)
        self.get_logger().warning(
            f"BLUE_CROSS: under the blue bar, {distance:.2f} m at "
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
        self.outcome = (f"COURSE COMPLETE: window, over red at "
                        f"{self.red_altitude:.2f} m, under blue at "
                        f"{self.blue_altitude:.2f} m")
        self._begin_landing("course complete")

    # ------------------------------------------------------ plumbing

    def rangefinder_is_healthy(self):
        # Only ever excused for the duration of the _still_flyable() call below.
        if self.rng_excused:
            return True
        return super().rangefinder_is_healthy()

    def _still_flyable(self):
        """Inherited, except a rangefinder dropout does not land us over red.

        Over the red bar the lidar sees the bar top instead of the floor, a
        ~2 m step that EKF2's kinematic check rejects. Landing there descends
        onto the bar. Instead the dropout is tolerated for
        course_rng_dropout_timeout: flow_is_healthy() also needs the
        rangefinder, so the existing open-loop push to P1 takes over, and
        BLUE_DROP (not excused) lands on P1 if fusion is still not back.
        Every other check, and every other stage, is untouched.
        """
        if (self.current_stage != self.RED_CROSS
                or super().rangefinder_is_healthy()):
            return super()._still_flyable()

        lost = 0.0 if self.rng_lost_since is None else time.monotonic() - self.rng_lost_since
        if lost > self.COURSE_RNG_DROPOUT_TIMEOUT:
            self._begin_landing(
                f"rangefinder fusion lost for {lost:.1f} s during the red crossing "
                f"(limit {self.COURSE_RNG_DROPOUT_TIMEOUT:.1f} s); altitude is unanchored")
            return False
        self.get_logger().warning(
            f"RED_CROSS: rangefinder not fused for {lost:.1f} s -- expected over "
            f"the bar; tolerating up to {self.COURSE_RNG_DROPOUT_TIMEOUT:.1f} s.",
            throttle_duration_sec=0.5)
        self.rng_excused = True
        try:
            return super()._still_flyable()
        finally:
            self.rng_excused = False

    def _rng_flags_summary(self):
        f = self.estimator_flags
        lp = self.local_position
        dist = "n/a" if lp is None else f"{lp.dist_bottom:.2f} m (valid={lp.dist_bottom_valid})"
        if f is None:
            return f"dist_bottom={dist}, no estimator_status_flags"
        return (f"dist_bottom={dist} rng_hgt={f.cs_rng_hgt} "
                f"rng_terrain={f.cs_rng_terrain} kin_consistent={f.cs_rng_kin_consistent} "
                f"fault={f.cs_rng_fault} stuck={f.cs_rng_stuck}")

    def _where_summary(self):
        alt = self.relative_altitude()
        along, cross = self._target_errors()
        s = f"stage {self.current_stage}, alt {'n/a' if alt is None else f'{alt:.2f}'} m"
        if self.current_stage in self.COURSE_STAGES and along is not None:
            s += f", {along:.2f} m to the target, {cross:+.2f} m off the line"
        return s

    def _log_rangefinder_fusion(self):
        """Log only: every change in rangefinder fusion, in every stage."""
        healthy = super().rangefinder_is_healthy()
        now = time.monotonic()
        if self.rng_was_healthy is None:
            self.rng_was_healthy = healthy
            if not healthy:
                self.rng_lost_since, self.rng_lost_stage = now, self.current_stage
            return
        if healthy != self.rng_was_healthy:
            if not healthy:
                self.rng_lost_since, self.rng_lost_stage = now, self.current_stage
                self.get_logger().error(
                    f"RNG FUSION LOST: {self._where_summary()}. {self._rng_flags_summary()}")
            else:
                lost = 0.0 if self.rng_lost_since is None else now - self.rng_lost_since
                self.get_logger().warning(
                    f"RNG FUSION BACK after {lost:.2f} s (lost in {self.rng_lost_stage}): "
                    f"{self._where_summary()}. {self._rng_flags_summary()}")
                self.rng_lost_since = None
            self.rng_was_healthy = healthy
        elif not healthy and self.rng_lost_since is not None:
            self.get_logger().info(
                f"RNG still not fused ({now - self.rng_lost_since:.1f} s): "
                f"{self._where_summary()}. {self._rng_flags_summary()}",
                throttle_duration_sec=0.5)

    def _clock_stages(self):
        # The crossings are excluded for the same reason the window traverse
        # is: a stopwatch must not start a descent over or under a bar. The
        # vertical stages are at midpoints, where a descent passes nothing.
        return super()._clock_stages() + (self.RED_RISE, self.BLUE_DROP)

    def timer_callback(self):
        self._log_rangefinder_fusion()
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

        {
            self.RED_RISE: self._handle_red_rise,
            self.RED_CROSS: self._handle_red_cross,
            self.BLUE_DROP: self._handle_blue_drop,
            self.BLUE_CROSS: self._handle_blue_cross,
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
        if self.current_stage in (self.RED_RISE, self.BLUE_DROP):
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
