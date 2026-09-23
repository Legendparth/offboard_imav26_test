#!/usr/bin/env python3
"""Watch the three things that die together when window_traverse loses Offboard.

WHY THIS EXISTS

    The failure in test_debugs.txt is that PX4 raises, in one message and at
    one instant, about a second after arming:

        local_position_invalid, local_velocity_invalid, offboard_control_signal_lost

    ... clears them ~1.2 s later, and raises them again 4.0 s after that. It
    does this whether or not the flight node is still streaming setpoints, so
    the flight node is not the cause. Three flags going bad together and
    recovering together means ONE upstream thing stopped, and there are only
    two candidates:

    (a) EKF2 stopped fusing the ZED (vibration off the motors breaks an
        IMU-less stereo VO), which invalidates position and velocity -- but
        would NOT touch the offboard heartbeat; or
    (b) the uXRCE-DDS serial link stalled, which starves the vision going IN
        and the setpoints going IN at the same time, and explains all three.

    (b) explains the lockstep and (a) does not, but (a) is the more common
    fault on this airframe. This tells you which, in one ground run.

HOW TO RUN IT -- PROPS OFF. Nothing here arms, commands or publishes anything;
it only listens. Start the normal stack, start this, then arm from the RC or
QGC and let it sit.

    ros2 launch drone_testing window_traverse.launch.py agent_only:=true
    python3 src/drone_testing/tools/vio_dropout_probe.py

    ... then arm, wait ~20 s, disarm.

READING IT

    cs_ev_pos goes false and the vision gap stays small
        -> EKF2 rejected the vision it was still receiving. Case (a):
           vibration. Fix the camera mount, not the code. Confirm by running
           it again with props ON, held down -- the dropout should get worse.

    the vision gap blows up (>1 s) at the same moment
        -> nothing was arriving to fuse. Case (b): the link. Raise
           SER_TEL2_BAUD and the agent's -b to 2000000, or cut what PX4
           bridges in dds_topics.yaml.

    neither, and it never drops with props off
        -> it is the motors. Same conclusion as (a).
"""

import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (EstimatorStatusFlags, FailsafeFlags, VehicleOdometry,
                          VehicleStatus)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)

WATCHED = ('local_position_invalid', 'local_velocity_invalid',
           'offboard_control_signal_lost', 'local_position_invalid_relaxed')


class Probe(Node):

    def __init__(self):
        super().__init__('vio_dropout_probe')
        self.t0 = time.monotonic()
        self.last_vision = None
        self.vision_gaps = []
        self.prev = {}

        self.create_subscription(EstimatorStatusFlags,
                                 '/uav_1/fmu/out/estimator_status_flags',
                                 self.flags_cb, SENSOR_QOS)
        self.create_subscription(FailsafeFlags, '/uav_1/fmu/out/failsafe_flags',
                                 self.failsafe_cb, SENSOR_QOS)
        self.create_subscription(VehicleStatus, '/uav_1/fmu/out/vehicle_status_v1',
                                 self.status_cb, SENSOR_QOS)
        # What the bridge is offering PX4. Note this is the ROS side of the
        # link: a message counted here has NOT necessarily crossed the serial
        # line. That is the point -- if these keep coming while cs_ev_pos goes
        # false, the loss is downstream of here.
        self.create_subscription(VehicleOdometry,
                                 '/uav_1/fmu/in/vehicle_visual_odometry',
                                 self.vision_cb, SENSOR_QOS)
        self.create_timer(1.0, self.tick)
        print("t=0 is now. Arm when ready (PROPS OFF). Ctrl-C to stop.\n")
        print(f"{'t':>7}  {'event':<44} vision")

    def stamp(self):
        return time.monotonic() - self.t0

    def say(self, event):
        gap = ('never' if self.last_vision is None
               else f"{(time.monotonic() - self.last_vision) * 1000:.0f} ms ago")
        print(f"{self.stamp():7.2f}  {event:<44} {gap}", flush=True)

    def changed(self, key, value):
        """True on a real transition. The first sight of a flag is reported
        as 'initial' rather than as a change, so the burst of lines when the
        first failsafe_flags message lands is not mistaken for a dropout."""
        first = key not in self.prev
        if not first and self.prev[key] == value:
            return False
        self.prev[key] = value
        return 'initial' if first else True

    def vision_cb(self, msg):
        now = time.monotonic()
        if self.last_vision is not None:
            self.vision_gaps.append(now - self.last_vision)
        self.last_vision = now

    def flags_cb(self, msg):
        for name in ('cs_ev_pos', 'cs_ev_vel', 'cs_ev_yaw', 'cs_yaw_align'):
            value = bool(getattr(msg, name, False))
            state = self.changed(name, value)
            if state:
                mark = '(initial)' if state == 'initial' else ''
                self.say(f"{name} -> {value} {mark}".rstrip())

    def failsafe_cb(self, msg):
        for name in WATCHED:
            value = bool(getattr(msg, name, False))
            state = self.changed(name, value)
            if state:
                mark = ' (initial)' if state == 'initial' else ''
                self.say(f"{'FAILSAFE' if value else 'cleared '} {name}{mark}")

    def status_cb(self, msg):
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if self.changed('armed', armed):
            self.say(f"*** {'ARMED' if armed else 'DISARMED'} ***")
            self.vision_gaps = []

    def tick(self):
        """Once a second, the vision rate over the last second."""
        gaps, self.vision_gaps = self.vision_gaps, []
        if not gaps:
            self.say("vision: NOTHING in the last second")
            return
        worst = max(gaps)
        if worst > 0.3:
            self.say(f"vision: {len(gaps)} msgs, worst gap {worst * 1000:.0f} ms")


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\nstopped.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
