#!/usr/bin/env python3
"""Find the servo, and prove it moves, before any of the rest of it.

The mission is a slow way to debug a servo. This node talks to one directly
and tells you which PX4 output it is on, which is the number everything else
then needs.

    command:=sweep      (the default) MAV_CMD_ACTUATOR_TEST across Servo 1-8,
                        one per second, printing PX4's ack for each. WATCH THE
                        SERVO. The number printed for the step that moves it
                        is what servo_function wants, exactly as printed.

                        It sweeps param5 1201-1208 first and falls back to
                        33-40 by itself if PX4 says UNSUPPORTED -- see the
                        numbering note further down, which is not obvious and
                        has cost a bench session already.

    command:=set        One output, over and over, so you can watch it and
                        adjust the horn:
                            -p command:=set -p function:=204 -p value:=1.0

    command:=toggle     The old behaviour of this file: MAV_CMD_DO_SET_ACTUATOR
                        flipping "Offboard Actuator Set <index>" between two
                        values every couple of seconds. This is the command
                        the MISSION uses in the air, so it is the one to test
                        once the aircraft is ARMED -- see the warning below.

WHY sweep AND toggle ARE NOT THE SAME TEST

    MAV_CMD_DO_SET_ACTUATOR (toggle) is what thermal_drop and
    servo_controller send in flight. PX4 applies it only while ARMED, and
    only to an output assigned to "Offboard Actuator Set N" in QGC ->
    Actuators. On a disarmed bench it can be accepted and still move nothing,
    which looks exactly like broken wiring and is not.

    MAV_CMD_ACTUATOR_TEST (sweep, set) is what the QGC Actuators sliders
    send. It works DISARMED, which is the only reason a bench test is
    possible at all, but it addresses the output by its FUNCTION number
    rather than by the offboard set.

    So: use sweep on the bench to find the output, then toggle (armed, props
    OFF) to prove the flight path itself works end to end.

IF PX4 REFUSES EVERYTHING WITH "DENIED"

    That is not the servo and not the wiring. Commander denies an actuator
    test outright, before it ever looks at the function number, for exactly
    three reasons:

        the vehicle is ARMED;
        a SAFETY BUTTON is fitted and has not been pressed -- the usual
            cause. Press it until the LED stops blinking;
        COM_MOT_TEST_EN is not 1 -- `param set COM_MOT_TEST_EN 1` in the
            PX4 console, or find it in QGC's parameter list.

    Nothing in this node can work around any of the three.

Run:
    ros2 run drone_testing servo_test --ros-args -p command:=sweep
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import VehicleCommand, VehicleCommandAck

ACTUATOR_TEST = 310         # MAV_CMD_ACTUATOR_TEST
DO_SET_ACTUATOR = 187       # MAV_CMD_DO_SET_ACTUATOR

# WHAT GOES IN param5, AND THE TRAP IN IT.
#
# param5 is a MAVLink ACTUATOR_OUTPUT_FUNCTION, which is NOT the same numbering
# as PX4's internal output functions, even though both call the outputs
# "Servo 1-8". Commander.cpp::handleCommandActuatorTest reads it as:
#
#     1 .. 12     motors 1-12      (MAVLink numbering)
#     33 .. 40    servos 1-8       (MAVLink numbering)
#     >= 1000     PX4's own internal function, minus 1000
#
# PX4's internal FUNCTION_SERVO1 is 201, so Servo 1-8 internally are 201-208 --
# and sending those RAW lands in neither range above and is answered
# UNSUPPORTED. They have to be sent as 1201-1208. Both encodings reach the same
# physical output; the 1000+ form is the one QGC's Actuators tab uses, so it is
# the default here and the alternate is the fallback.
PX4_SERVO_FUNCTIONS = list(range(1201, 1209))       # 1000 + FUNCTION_SERVO1..8
MAVLINK_SERVO_FUNCTIONS = list(range(33, 41))       # ACTUATOR_OUTPUT_FUNCTION

# MAV_RESULT, so a refusal says what it means rather than a bare number.
RESULTS = {0: 'ACCEPTED', 1: 'TEMPORARILY REJECTED', 2: 'DENIED',
           3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN PROGRESS', 6: 'CANCELLED'}

DENIED_HELP = (
    "DENIED is PX4 refusing before it even looks at the function number. "
    "handleCommandActuatorTest denies for exactly three reasons: the vehicle "
    "is ARMED; a SAFETY BUTTON is fitted and has not been pressed (this is "
    "the usual one -- press it until the LED stops blinking); or "
    "COM_MOT_TEST_EN is not 1 (`param set COM_MOT_TEST_EN 1`, or set it in "
    "QGC). Nothing in this node can work around any of the three.")


class ServoTestNode(Node):

    def __init__(self):
        super().__init__('servo_test_node')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.COMMAND = str(p('command', 'sweep')).lower()
        self.VALUE = float(p('value', 1.0))
        self.NEUTRAL = float(p('neutral', -1.0))
        self.FUNCTION = int(p('function', 0))
        self.INDEX = int(p('index', 1))
        self.STEP_SECONDS = float(p('step_seconds', 1.0))
        self.FUNCTIONS = [int(f) for f in p('functions', PX4_SERVO_FUNCTIONS)]

        self.publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # PX4's /fmu/out/... topics are BEST_EFFORT. A RELIABLE subscriber is
        # matched with nothing and silently never sees an ack, which would
        # make every output here look equally dead.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.ack_callback, px4_qos)

        self.position = self.NEUTRAL
        self.step = 0
        self.acks = 0
        self.testing = None     # the function currently under test, for acks
        self.results = {}       # function -> MAV_RESULT, or None if unanswered
        self.tried_alternate = False

        if self.COMMAND == 'sweep':
            self.timer = self.create_timer(self.STEP_SECONDS, self.sweep_callback)
            self.get_logger().warning(
                f"SWEEP: MAV_CMD_ACTUATOR_TEST at {self.VALUE:+.2f} across "
                f"functions {self.FUNCTIONS[0]}-{self.FUNCTIONS[-1]} "
                f"(Servo 1-{len(self.FUNCTIONS)}), {self.STEP_SECONDS:.1f} s "
                "each. WATCH THE SERVO and note which function moves it.")
        elif self.COMMAND == 'set':
            if self.FUNCTION == 0:
                self.get_logger().error(
                    "command:=set needs -p function:=<N>, the PX4 output "
                    "FUNCTION number. Run command:=sweep to find it.")
            self.timer = self.create_timer(0.1, self.set_callback)
            self.get_logger().warning(
                f"SET: function {self.FUNCTION} held at {self.VALUE:+.2f} "
                "via MAV_CMD_ACTUATOR_TEST.")
        elif self.COMMAND == 'toggle':
            self.timer = self.create_timer(0.1, self.toggle_callback)
            self.start_time = time.monotonic()
            self.get_logger().warning(
                f"TOGGLE: MAV_CMD_DO_SET_ACTUATOR on offboard actuator set "
                f"{self.INDEX}, flipping {self.NEUTRAL:+.2f}/{self.VALUE:+.2f} "
                "every 2 s. THIS ONLY MOVES ANYTHING WHILE ARMED -- props off.")
        else:
            self.get_logger().error(
                f"Unknown command '{self.COMMAND}'. Use sweep, set or toggle.")
            self.timer = None

    # ------------------------------------------------------------- the modes

    def sweep_callback(self):
        if self.step >= len(self.FUNCTIONS):
            self.finish_pass()
            return
        fn = self.FUNCTIONS[self.step]
        self.step += 1
        self.testing = fn
        self.results[fn] = None
        # param2 is the test timeout: PX4 releases the output when it expires,
        # so each step tidies up after itself. PX4 caps it at 3 s.
        self.send_actuator_test(fn, self.VALUE, self.STEP_SECONDS)
        self.get_logger().info(
            f"  {self.label(fn)}  ->  {self.VALUE:+.2f}")

    def label(self, fn):
        if fn >= 1000:
            return f"param5 {fn}  (PX4 function {fn - 1000} = Servo {fn - 1200})"
        if 33 <= fn <= 40:
            return f"param5 {fn}  (MAVLink Servo {fn - 32})"
        return f"param5 {fn}"

    def finish_pass(self):
        """End of one encoding's pass. Decide whether to try the other."""
        self.timer.cancel()
        seen = [r for r in self.results.values() if r is not None]
        unsupported = [r for r in seen if r == 3]
        denied = [r for r in seen if r == 2]

        if not seen:
            self.get_logger().error(
                f"Not one ack from PX4 across {len(self.results)} outputs. The "
                "commands are not arriving at all: check that the uXRCE-DDS "
                "agent is running and connected (`ros2 topic hz "
                "/fmu/out/vehicle_status`). Nothing here was refused, because "
                "nothing here was heard.")
            return

        if denied:
            self.get_logger().error(
                f"PX4 DENIED {len(denied)} of {len(seen)} outputs. " + DENIED_HELP)
            return

        if unsupported and len(unsupported) == len(seen) and not self.tried_alternate:
            # Right idea, wrong numbering scheme. Try the other one rather
            # than making someone read the MAVLink spec on a bench.
            self.tried_alternate = True
            self.FUNCTIONS = (MAVLINK_SERVO_FUNCTIONS
                              if self.FUNCTIONS[0] >= 1000 else PX4_SERVO_FUNCTIONS)
            self.step = 0
            self.results = {}
            self.get_logger().warning(
                "Every output came back UNSUPPORTED, which means the numbering "
                "was wrong rather than the servo. Retrying with the other "
                f"encoding: param5 {self.FUNCTIONS[0]}-{self.FUNCTIONS[-1]}.")
            self.timer = self.create_timer(self.STEP_SECONDS, self.sweep_callback)
            return

        accepted = [fn for fn, r in self.results.items() if r == 0]
        if accepted:
            self.get_logger().warning(
                f"Sweep finished. PX4 ACCEPTED {len(accepted)} output(s): "
                + ', '.join(str(f) for f in accepted) + ". "
                "Whichever one MOVED the servo is the number to pass as "
                "servo_function -- pass it exactly as printed above, it is "
                "already in the encoding PX4 wants. If PX4 accepted them all "
                "and NOTHING moved, the servo is not on Servo 1-8, or the "
                "servo rail is not powered (a Pixhawk does not power it from "
                "the FMU -- it needs BEC voltage on the rail).")
        else:
            self.get_logger().error(
                "Sweep finished with no output accepted. Results: "
                + ', '.join(f"{fn}={RESULTS.get(r, r)}"
                            for fn, r in self.results.items() if r is not None))

    def set_callback(self):
        self.testing = self.FUNCTION
        self.send_actuator_test(self.FUNCTION, self.VALUE, 1.0)

    def toggle_callback(self):
        if time.monotonic() - self.start_time > 2.0:
            self.position = (self.VALUE if self.position == self.NEUTRAL
                             else self.NEUTRAL)
            self.start_time = time.monotonic()
            self.get_logger().info(
                f"Offboard actuator set {self.INDEX} -> {self.position:+.2f}")

        nan = float('nan')
        params = [nan] * 6          # NaN leaves the other outputs alone
        params[max(1, min(6, self.INDEX)) - 1] = self.position
        msg = VehicleCommand()
        msg.command = DO_SET_ACTUATOR
        (msg.param1, msg.param2, msg.param3,
         msg.param4, msg.param5, msg.param6) = params
        msg.param7 = 0.0
        self.stamp_and_publish(msg)

    # ---------------------------------------------------------------- the wire

    def send_actuator_test(self, function, value, timeout):
        msg = VehicleCommand()
        msg.command = ACTUATOR_TEST
        msg.param1 = float(value)
        msg.param2 = float(timeout)
        msg.param3 = msg.param4 = 0.0
        msg.param5 = float(function)
        msg.param6 = msg.param7 = 0.0
        self.stamp_and_publish(msg)

    def stamp_and_publish(self, msg):
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher.publish(msg)

    def ack_callback(self, msg):
        if msg.command not in (ACTUATOR_TEST, DO_SET_ACTUATOR):
            return
        self.acks += 1
        name = RESULTS.get(msg.result, f"result {msg.result}")
        if self.COMMAND == 'sweep':
            # No throttling here. One ack per step, and a suppressed one reads
            # as an output that never answered, which is a different fault.
            if self.testing is not None:
                self.results[self.testing] = msg.result
            # Two call sites on purpose: rclpy keys its logger cache on the
            # source location, and one line that alternates between .info and
            # .error raises "Logger severity cannot be changed between calls".
            if msg.result == 0:
                self.get_logger().info(f"    PX4 {name}")
            else:
                self.get_logger().error(f"    PX4 {name}")
            return
        where = (f"function {self.testing}" if self.testing
                 else f"actuator set {self.INDEX}")
        if msg.result == 0:
            self.get_logger().info(f"    PX4 {name}: {where}",
                                   throttle_duration_sec=2.0)
        else:
            self.get_logger().error(f"    PX4 {name}: {where}",
                                    throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ServoTestNode()
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
