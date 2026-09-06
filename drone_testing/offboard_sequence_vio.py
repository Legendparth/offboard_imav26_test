"""
The sequence test, flown on ZED visual odometry instead of optical flow.

Same mission, same stages, same keyboard aborts as offboard_sequence:

    arm -> sit on the ground -> climb -> hold -> run the commanded motions
    one at a time, holding between each -> hold -> land

This node subclasses offboard_sequence.OffboardSequence and changes exactly
one thing: WHERE THE LATERAL ESTIMATE COMES FROM, and therefore what counts
as "healthy enough to fly a position setpoint". Everything else -- the ramps,
the leashes, the EKF2-reset bookkeeping, the touchdown fallback, the failsafe
reporting -- is inherited, because none of it cares which sensor produced x
and y.

WHY A SUBCLASS AND NOT A COPY
-----------------------------
The base class already funnels every horizontal decision through one
predicate, flow_is_healthy(). Overriding that predicate (and the settle time
around it) moves the entire node onto vision without touching a single line
of flight logic, which means the flow version and the vision version cannot
drift apart as the state machine is fixed. The predicate keeps its original
name for the same reason -- renaming it would fork the base class.

WHAT ACTUALLY CHANGES IN THE AIR
--------------------------------
1. The health test. Optical flow is gated on the rangefinder and on being
   above FLOW_MIN_AGL, because flow over a floor 15 cm away sees nothing.
   Vision has no such floor: the ZED is looking outward at a room. So the
   AGL gate is dropped and replaced by a test that EKF2 is really fusing the
   external-vision estimate (cs_ev_pos / cs_ev_vel, no cs_ev_fault) plus a
   freshness signal from the bridge node itself.

2. x/y position hold latches ON THE GROUND. This is the real prize. The flow
   airframe has no usable lateral estimate until it is airborne, so the base
   class flies the ground wait and the whole climb as "zero velocity and
   hope", and can only latch a hold point once it is up. Vision is valid
   while the vehicle is sitting still on the floor, so the climb can be flown
   as a genuine position hold against the take-off point, and the vehicle
   goes straight up instead of sliding off. Set hold_xy_from_ground:=false to
   get the old behaviour back.

3. Losing vision mid-flight is survivable, not fatal. If EKF2 stops fusing
   the vision the inherited logic drops back to zero-velocity hold on its own
   and horizontal steps are SKIPPED rather than dead-reckoned. The height
   estimate is still the lidar's, so the descent is unaffected -- which is
   the whole reason for keeping the rangefinder as the height reference
   rather than letting vision own z as well.

WHAT DOES NOT CHANGE
--------------------
The rangefinder is still a hard arming gate and still a hard in-flight gate,
because it is still the height reference. Vision that dies is a lost mission;
a height estimate that dies is a lost aircraft.

PX4 SIDE
--------
This node assumes the parameters listed in launch/sequence_vio_test.launch.py
are already set on the flight controller, and it will not arm if EKF2 is not
actually fusing what it thinks it is. Read that file before the first flight.

Keys:  q -> abort into a controlled descent.  k -> force-disarm.
"""

import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_testing.offboard_sequence import OffboardSequence


class OffboardSequenceVio(OffboardSequence):

    # x/y hold is latched after the vision estimate has been continuously
    # healthy this long. Longer than the flow version's 1.0 s: a VO estimate
    # that has just recovered tracking is at its least trustworthy, and there
    # is no time pressure here because vision is available on the ground.
    FLOW_SETTLE_SECONDS = 2.0

    # Anchor x/y to the take-off point during the ground wait and the climb,
    # rather than waiting until level flight. See point 2 above.
    HOLD_XY_FROM_GROUND = True

    # How stale the bridge's health heartbeat may be before we stop believing
    # it. The bridge publishes at 10 Hz; three missed messages is plenty.
    VIO_STATUS_TIMEOUT = 0.5    # s

    # If the bridge is not publishing vio_healthy at all, fall back to EKF2's
    # flags alone. True is the safe default -- the flags are the authoritative
    # answer to "is this being fused", and a missing bridge topic is a launch
    # mistake, not a sensor failure. Set false to make the topic mandatory.
    ALLOW_MISSING_BRIDGE_STATUS = True

    def __init__(self, node_name='offboard_sequence_vio'):
        super().__init__(node_name)

        self.HOLD_XY_FROM_GROUND = bool(self.declare_parameter(
            'hold_xy_from_ground', self.HOLD_XY_FROM_GROUND).value)
        self.FLOW_SETTLE_SECONDS = float(self._declare_number(
            'vio_settle_seconds', self.FLOW_SETTLE_SECONDS))
        self.ALLOW_MISSING_BRIDGE_STATUS = bool(self.declare_parameter(
            'allow_missing_bridge_status', self.ALLOW_MISSING_BRIDGE_STATUS).value)

        # The bridge's own view of whether ZED odometry is arriving. Matched
        # to the publisher in zed_localization.py: reliable + transient-local,
        # so this arrives even if the bridge came up first.
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vio_healthy = False
        self.vio_status_time = None
        self.vio_healthy_sub = self.create_subscription(
            Bool, '/vio_healthy', self.vio_healthy_callback, status_qos)

        # Reported once when it first goes bad, so a launch with the bridge
        # missing is loud rather than mysterious.
        self._warned_no_bridge = False

        if self.__class__ is not OffboardSequenceVio:
            return

        plan = " -> ".join(str(s) for s in self.steps)
        self.get_logger().warning(
            f"Sequence test on ZED VISION: climb {self.TAKEOFF_ALTITUDE:.2f} m, "
            f"hold {self.HOLD_SECONDS:.0f} s, then {len(self.steps)} steps: "
            f"{plan}, hold {self.POST_HOLD_SECONDS:.0f} s, land. Directions are "
            f"in the '{self.DIRECTION_FRAME}' yaw frame. x/y hold "
            f"{'starts on the ground' if self.HOLD_XY_FROM_GROUND else 'waits for altitude'}. "
            "Press q to abort into a descent, k to force-disarm.")

    # ------------------------------------------------------------------ subs

    def vio_healthy_callback(self, msg):
        self.vio_healthy = bool(msg.data)
        self.vio_status_time = time.monotonic()

    def bridge_is_alive(self):
        """Is zed_localization publishing, and does it say the ZED is alive?"""
        if self.vio_status_time is None:
            return self.ALLOW_MISSING_BRIDGE_STATUS
        if time.monotonic() - self.vio_status_time > self.VIO_STATUS_TIMEOUT:
            return self.ALLOW_MISSING_BRIDGE_STATUS
        return self.vio_healthy

    # ------------------------------------------------------------- estimator

    def vision_is_fused(self):
        """Is EKF2 actually fusing the external-vision estimate right now?

        This is the vision equivalent of rangefinder_is_healthy(), and it is
        asked for the same reason: xy_valid tells you the estimator is willing
        to publish a number, not that anything is correcting it. EKF2 keeps
        xy_valid true for a good while after vision stops arriving, coasting
        on integrated accelerometer data, and a position setpoint flown
        against a coasting estimate walks the aircraft across the room.

        cs_ev_pos / cs_ev_vel are the flags EKF2 sets when external-vision
        aiding is actually selected. Either one is enough -- which of them is
        set depends on the EKF2_EV_CTRL bitmask and on whether the bridge is
        publishing velocity.

        The only EV fault flag this px4_msgs actually carries is
        cs_ev_yaw_fault; the position and velocity ones are read through
        getattr with a False default so that a firmware/px4_msgs pair that
        does gain them starts being checked automatically, and one that never
        does keeps working. Note what this means today: a vision POSITION that
        EKF2 has started rejecting shows up here only when cs_ev_pos finally
        clears, which is why the bridge's own freshness heartbeat is part of
        the test rather than a nicety.

        With no estimator flags being published at all there is no second
        opinion available, so we fall back to the bridge heartbeat plus
        xy_valid. That is weaker, and it is why the launch file leaves the
        estimator_status_flags topic switched on.
        """
        f = self.estimator_flags
        if f is None:
            return None
        aiding = getattr(f, 'cs_ev_pos', False) or getattr(f, 'cs_ev_vel', False)
        faulted = (getattr(f, 'cs_ev_pos_fault', False)
                   or getattr(f, 'cs_ev_vel_fault', False)
                   or getattr(f, 'cs_ev_yaw_fault', False))
        return bool(aiding) and not faulted

    def flow_is_healthy(self):
        """Overridden: 'is the LATERAL estimate trustworthy', vision edition.

        Keeps the base class's name so every inherited gate -- the x/y latch,
        the decision to run or skip a horizontal step, the status line --
        switches to vision with no other change.

        Deliberately does NOT test dist_bottom or FLOW_MIN_AGL. Those exist
        because optical flow cannot see a floor that is 15 cm from the lens;
        the ZED is looking out across a room and works fine sitting on the
        ground, which is exactly the property this node is here to exploit.
        """
        lp = self.local_position
        if lp is None or not lp.xy_valid or not lp.v_xy_valid:
            return False
        if not self.bridge_is_alive():
            return False
        fused = self.vision_is_fused()
        if fused is None:
            # No estimator flags. xy_valid plus a live bridge is all we have.
            return True
        return fused

    # -------------------------------------------------------- early x/y hold

    def _handle_ground_wait(self):
        # Latch before the base class runs, so the very first setpoint of the
        # climb is already a position hold rather than a zero-velocity one.
        if self.HOLD_XY_FROM_GROUND:
            self._try_latch_xy_hold()
        super()._handle_ground_wait()

    def _handle_takeoff(self):
        if self.HOLD_XY_FROM_GROUND:
            self._try_latch_xy_hold()
        super()._handle_takeoff()

    def _try_latch_xy_hold(self):
        # The base class dereferences local_position unconditionally once the
        # health test passes. On the ground that test can now pass earlier
        # than the first VehicleLocalPosition, so guard it here.
        if self.local_position is None:
            return
        super()._try_latch_xy_hold()

    # -------------------------------------------------------------- logging

    def log_flight_state(self):
        super().log_flight_state()
        fused = self.vision_is_fused()
        self.get_logger().info(
            f"vio: bridge={'ok' if self.bridge_is_alive() else 'BAD'} "
            f"ekf_fusing={'?' if fused is None else fused} "
            f"healthy={self.flow_is_healthy()} "
            f"xy={'POS-HOLD' if self.hold_xy else 'VEL-HOLD'}",
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OffboardSequenceVio()
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
