"""
Take the rangefinder away from EKF2, mid-course, on purpose. SITL ONLY.

    ros2 launch drone_testing course_mission_sitl.launch.py rng_dropout:=true

WHAT IT REPRODUCES

    On the hardware flights the lidar stops being fused just after the red
    bar. The cause is mechanical, not electrical: crossing the bar steps
    dist_bottom by the bar's height in a single frame, EKF2's kinematic
    consistency check compares d(dist_bottom)/dt against its own vz, finds
    they disagree wildly, and clears cs_rng_kin_consistent. No range
    measurement is fused after that, and the flag can only be re-earned at
    |vz| > 0.5 m/s -- which is exactly what a vehicle holding position does
    not have. In the logs it came back after about ten seconds.

    The simulator does not reproduce it by itself: the bars are thin, the
    beam mostly misses them, and the world is scaled. So this node creates
    the same OUTCOME -- EKF2 fusing no range for a few seconds, in the place
    the hardware loses it -- by setting EKF2_RNG_CTRL to 0 and back.

    It is the outcome, not the mechanism: the consistency flag is not what
    goes false here, fusion is simply switched off. For testing what the
    state machine does about it (hold position, hold height on the baro,
    confirm the fusion is steady before moving on) that is the same thing.

HOW IT KNOWS WHEN

    It reads course_fsm's own status topic and waits for the stage this is
    aimed at -- RED_CROSS by default, the level flight over the red bar --
    then waits trigger_delay seconds so the drop lands with the aircraft
    over or just past the bar rather than at the moment it starts moving.

HOW IT TALKS TO PX4

    The px4-param client binary, over the running instance's socket in /tmp.
    That is the same route as typing "param set" at a pxh> prompt, and it
    needs no MAVLink, no parameter service and no changes to the flight
    node. It only works on the machine the SITL instance is running on --
    which is the only place this node is ever meant to run.

WHAT IT DOES NOT DO

    Nothing on this node affects the aircraft directly. It sets one
    estimator parameter and sets it back; if it dies in between, the
    parameter stays cleared, so the restore is also run on shutdown.
"""

import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RngDropout(Node):

    def __init__(self):
        super().__init__('rng_dropout')

        self.status_topic = str(self.declare_parameter(
            'status_topic', '/takeoff_status').value)
        self.stage = str(self.declare_parameter('trigger_stage', 'RED_CROSS').value)
        self.delay = float(self.declare_parameter('trigger_delay', 2.0).value)
        self.seconds = float(self.declare_parameter('dropout_seconds', 10.0).value)
        self.param = str(self.declare_parameter('param', 'EKF2_RNG_CTRL').value)
        self.gone_value = str(self.declare_parameter('dropout_value', '0').value)
        self.back_value = str(self.declare_parameter('restore_value', '2').value)
        self.px4_param = str(self.declare_parameter(
            'px4_param_bin',
            '~/PX4-Autopilot/build/px4_sitl_default/bin/px4-param').value)
        self.repeat = bool(self.declare_parameter('repeat', False).value)

        self.armed_at = None        # monotonic time the trigger stage began
        self.dropped_at = None      # ... and when the parameter went away
        self.done = False

        self.create_subscription(String, self.status_topic, self.status_callback, 10)
        self.create_timer(0.2, self.tick)

        self.get_logger().warning(
            f"SITL RANGEFINDER DROPOUT ARMED. {self.delay:.1f} s after "
            f"{self.stage} begins, {self.param} goes to {self.gone_value} for "
            f"{self.seconds:.1f} s and then back to {self.back_value}. This is "
            "a deliberate fault injection -- it must never be launched "
            "against a real aircraft.")

    # ------------------------------------------------------------- triggers

    def status_callback(self, msg):
        """stage|armed|altitude|flow|detail -- only the stage matters here."""
        stage = msg.data.split('|')[0].strip()
        if stage != self.stage or self.armed_at is not None or self.done:
            return
        self.armed_at = time.monotonic()
        self.get_logger().warning(
            f"{self.stage} has begun. Taking the rangefinder away in "
            f"{self.delay:.1f} s.")

    def tick(self):
        now = time.monotonic()
        if self.dropped_at is not None:
            if now - self.dropped_at >= self.seconds:
                self.restore()
            return
        if self.done or self.armed_at is None:
            return
        if now - self.armed_at >= self.delay:
            self.drop()

    # --------------------------------------------------------------- the act

    def _px4_param(self, *args):
        cmd = f"{self.px4_param} " + " ".join(args)
        try:
            out = subprocess.run(cmd, shell=True, capture_output=True,
                                 text=True, timeout=5.0)
        except subprocess.SubprocessError as exc:
            self.get_logger().error(f"px4-param failed: {exc}")
            return False
        if out.returncode != 0:
            self.get_logger().error(
                f"px4-param returned {out.returncode}: "
                f"{(out.stderr or out.stdout).strip()}. Is this the machine "
                "the SITL instance is running on?")
            return False
        return True

    def drop(self):
        if not self._px4_param('set', self.param, self.gone_value):
            self.done = True
            return
        self.dropped_at = time.monotonic()
        self.get_logger().error(
            f"RANGEFINDER GONE: {self.param}={self.gone_value}. EKF2 is fusing "
            f"no range for the next {self.seconds:.1f} s -- height is the "
            "barometer's now. The aircraft should hold this position, dead "
            "still, and not move on until the fusion is back and has stayed "
            "back.")

    def restore(self):
        self.dropped_at = None
        self.done = not self.repeat
        if self.repeat:
            self.armed_at = None
        if self._px4_param('set', self.param, self.back_value):
            self.get_logger().warning(
                f"RANGEFINDER BACK: {self.param}={self.back_value}. EKF2 will "
                "take a moment to re-establish the fusion; course_fsm waits "
                "for it to stay healthy before it moves.")

    def destroy_node(self):
        # Never leave the estimator crippled because this node stopped.
        if self.dropped_at is not None:
            self._px4_param('set', self.param, self.back_value)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RngDropout()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
