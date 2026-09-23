#!/usr/bin/env python3
"""Count how often EKF2 resets its position estimate. No arming, no flying.

WHY THIS EXISTS

    The window_traverse failure -- PX4 raising local_position_invalid a second
    or so after arming -- is NOT caused by arming. EKF2 is resetting its
    horizontal position six to seven times a SECOND, continuously, while the
    vehicle sits disarmed and motionless. Arming is only when PX4 starts
    enforcing position validity, so it is when a fault that was there all
    along becomes a failsafe.

    That matters because it turns a dangerous, slow, arm-it-and-see experiment
    into a 25-second bench measurement. Change one thing, run this, compare
    the number. A healthy vehicle sitting still should read approximately
    ZERO resets per second.

USAGE

    ros2 launch drone_testing window_traverse.launch.py agent_only:=true
    python3 src/drone_testing/tools/ekf_reset_rate.py [seconds]

    If it reports 0 messages, the uXRCE-DDS link is not up -- restart the
    launch and give the agent time to handshake before blaming the estimator.

WHAT TO VARY

    Each of these is one run. Note the resets/s each time.

    bridge:=false zed:=false     No vision reaching PX4 at all. THE baseline:
                                 if the resets continue with nothing feeding
                                 EKF2, the fault is in the PX4 parameter set
                                 and not in anything this repo publishes.
    publish_rate:=5 / 10 / 30    Does the reset rate track the vision rate?
    use_sample_timestamp:=true   Capture stamp vs one clock in the odometry.
                                 Measured at 7.08 vs 6.20 resets/s -- i.e.
                                 not the cause, recorded here so nobody
                                 spends another evening on it.
    pose_frame:=frd              Only meaningful with the magnetometer ON and
                                 EKF2_EV_CTRL=1; see README 10.3 first.

    And in QGC, one at a time: EKF2_EV_DELAY, EKF2_EV_CTRL, EKF2_HGT_REF,
    EKF2_EVP_NOISE, EKF2_EV_NOISE_MD.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleLocalPosition

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=1)


class ResetRate(Node):

    def __init__(self):
        super().__init__('ekf_reset_rate')
        self.messages = 0
        self.xy_resets = 0
        self.z_resets = 0
        self.last_xy = None
        self.last_z = None
        self.biggest = 0.0
        self.create_subscription(VehicleLocalPosition,
                                 '/uav_1/fmu/out/vehicle_local_position_v1',
                                 self.cb, SENSOR_QOS)

    def cb(self, msg):
        self.messages += 1
        if self.last_xy is not None and msg.xy_reset_counter != self.last_xy:
            self.xy_resets += 1
            # delta_xy is how far the estimate actually moved. Near zero means
            # EKF2 re-anchored to essentially the same place -- it is not
            # correcting drift, it is failing to fuse and starting over.
            self.biggest = max(self.biggest, abs(msg.delta_xy[0]),
                               abs(msg.delta_xy[1]))
        if self.last_z is not None and msg.z_reset_counter != self.last_z:
            self.z_resets += 1
        self.last_xy = msg.xy_reset_counter
        self.last_z = msg.z_reset_counter


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
    rclpy.init()
    node = ResetRate()
    print(f"measuring for {seconds:.0f} s -- keep the vehicle still...")
    end = time.time() + seconds
    try:
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    if node.messages == 0:
        print("no VehicleLocalPosition at all -- the uXRCE-DDS link is down.")
    else:
        print(f"{node.messages} messages in {seconds:.0f} s "
              f"({node.messages / seconds:.0f} Hz)")
        print(f"  horizontal resets : {node.xy_resets:4d}  "
              f"({node.xy_resets / seconds:.2f}/s)   "
              f"largest delta_xy {node.biggest:.3f} m")
        print(f"  vertical resets   : {node.z_resets:4d}  "
              f"({node.z_resets / seconds:.2f}/s)")
        print("\nA vehicle sitting still should read ~0.00/s on both.")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
