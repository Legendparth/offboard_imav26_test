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

    command:=cycle      FULL OPEN, FULL CLOSED, five times, then stop and go
                        quiet. For proving the release travels its whole range
                        repeatably and comes back to rest -- a servo that
                        drops the cone once and then stalls half open is a
                        failed mission, and one pass of `set` will not show it.

                            ros2 run drone_testing servo_test --ros-args \
                                -p command:=cycle -p function:=1301

                        It ends CLOSED and stops sending, so the bay is left
                        the way the mission expects to find it.

THIS AIRCRAFT'S ACTUATOR CONFIGURATION, read off QGC -> Actuators (MAIN 5-8)

    MAIN 5   Peripheral via Actuator Set 1   Disarmed 2000  Min 1000  Max 2000
                                             Rev Range: CHECKED

    Three things follow from that row, and all three matter here:

    1. The output is assigned "Peripheral via Actuator Set 1", NOT "Servo N".
       That is what servo_index:=1 addresses with MAV_CMD_DO_SET_ACTUATOR, and
       it is why the mission's set_actuator path is pointed at index 1.

    2. It also means a sweep over Servo 1-8 (param5 1201-1208 / 33-40) WILL
       NOT FIND IT, because the output does not carry a Servo function at all.
       The sweep therefore has a third pass over the offboard-actuator-set
       functions -- see FUNCTION ENCODINGS below. If you have been sweeping
       Servo 1-8 and finding nothing, this is why.

    3. REV RANGE IS CHECKED, so the actuator value maps to PWM BACKWARDS:
       -1 -> Maximum (2000 us) and +1 -> Minimum (1000 us). The disarmed value
       is 2000, which is therefore the position -1.0 commands. So:

            -1.0  = 2000 us = WHERE THE SERVO SITS DISARMED = closed/at rest
            +1.0  = 1000 us = the other end of its travel   = open

       which is exactly the servo_neutral_value / servo_drop_value pair the
       mission already uses, and the reason servo_close_on_start can claim the
       bay is "known-closed": -1.0 is the position the output holds anyway
       when nothing is commanding it.

       If your horn is mounted so that the disarmed position is OPEN, do not
       fix it by swapping these numbers -- fix the horn, or untick Rev Range.
       A bay that springs open the moment the FC is disarmed or reboots is a
       cone on the floor.

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
#
# There is a THIRD set that matters on this aircraft, because its release is
# assigned "Peripheral via Actuator Set 1" rather than "Servo N": PX4's
# offboard actuator set functions, believed to start at 301, so 1301-1306 in
# the 1000+ encoding. "Believed" is doing real work in that sentence -- the
# number is not one to take on trust, which is why the sweep SENDS it and lets
# PX4's ack decide rather than asserting it. An UNSUPPORTED answer here means
# the base is not 301 on your firmware, and nothing is lost by having asked.
PX4_SERVO_FUNCTIONS = list(range(1201, 1209))       # 1000 + FUNCTION_SERVO1..8
MAVLINK_SERVO_FUNCTIONS = list(range(33, 41))       # ACTUATOR_OUTPUT_FUNCTION
PX4_OFFBOARD_SET_FUNCTIONS = list(range(1301, 1307))  # 1000 + Offboard Set 1..6

# Tried in this order, each pass falling through to the next only when PX4
# answers UNSUPPORTED to every output in it.
SWEEP_ENCODINGS = [PX4_SERVO_FUNCTIONS, MAVLINK_SERVO_FUNCTIONS,
                   PX4_OFFBOARD_SET_FUNCTIONS]

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

        # ---- command:=cycle ----
        self.CYCLES = int(p('cycles', 5))
        self.HOLD_SECONDS = float(p('hold_seconds', 2.0))
        # THE TWO ENDS OF THE TRAVEL. Defaults from the Actuators tab, where
        # Rev Range is checked and the disarmed value is the Maximum -- so
        # -1.0 is the position the output already holds when nothing is
        # commanding it, and +1.0 is the far end. See the module docstring.
        self.OPEN_VALUE = float(p('open_value', 1.0))
        self.CLOSED_VALUE = float(p('closed_value', -1.0))
        # actuator_test works DISARMED and is addressed by FUNCTION;
        # set_actuator is what the mission sends and needs the vehicle ARMED.
        self.METHOD = str(p('method', 'actuator_test')).lower()

        self.publisher = self.create_publisher(
            VehicleCommand, '/uav_1/fmu/in/vehicle_command', 10)

        # PX4's /uav_1/fmu/out/... topics are BEST_EFFORT. A RELIABLE subscriber is
        # matched with nothing and silently never sees an ack, which would
        # make every output here look equally dead.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommandAck, '/uav_1/fmu/out/vehicle_command_ack',
                                 self.ack_callback, px4_qos)

        self.position = self.NEUTRAL
        self.step = 0
        self.acks = 0
        self.refusals = 0
        self.last_refusal = None
        self.testing = None     # the function currently under test, for acks
        self.results = {}       # function -> MAV_RESULT, or None if unanswered
        self.encodings = [e for e in SWEEP_ENCODINGS if e != self.FUNCTIONS]

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
        elif self.COMMAND == 'cycle':
            self.cycle_steps = [('closed, settling', self.CLOSED_VALUE)]
            for i in range(max(1, self.CYCLES)):
                self.cycle_steps.append(
                    (f"OPEN  {i + 1}/{self.CYCLES}", self.OPEN_VALUE))
                self.cycle_steps.append(
                    (f"CLOSED {i + 1}/{self.CYCLES}", self.CLOSED_VALUE))
            self.cycle_step = -1        # -1 so the first tick enters step 0
            self.cycle_next = 0.0       # when the current hold expires
            self.cycle_done = False
            if self.METHOD == 'actuator_test' and self.FUNCTION == 0:
                self.get_logger().error(
                    "command:=cycle with the default method needs "
                    "-p function:=<N>, the PX4 output FUNCTION number. Run "
                    "command:=sweep to find it -- on this aircraft the "
                    "release is on 'Peripheral via Actuator Set 1', so try "
                    "1301 first and note that a Servo 1-8 sweep will not "
                    "find it. Or use -p method:=set_actuator -p index:=1, "
                    "which addresses the offboard set directly but only "
                    "moves anything while ARMED.")
                self.timer = None
                return
            # 10 Hz, not once per phase: MAV_CMD_ACTUATOR_TEST carries a
            # timeout after which PX4 hands the output back to its disarmed
            # value, so a held position has to be RE-ASSERTED or the servo
            # springs shut in the middle of the hold and the test measures
            # the timeout rather than the servo.
            self.timer = self.create_timer(0.1, self.cycle_callback)
            self.get_logger().warning(
                f"CYCLE: {self.CYCLES} full open/close cycles, "
                f"{self.HOLD_SECONDS:.1f} s at each end, "
                f"{self.CLOSED_VALUE:+.2f} (closed, the disarmed rest "
                f"position) <-> {self.OPEN_VALUE:+.2f} (open), via "
                + (f"MAV_CMD_ACTUATOR_TEST on function {self.FUNCTION} "
                   "-- works disarmed."
                   if self.METHOD == 'actuator_test' else
                   f"MAV_CMD_DO_SET_ACTUATOR on offboard actuator set "
                   f"{self.INDEX} -- ONLY MOVES WHILE ARMED, props off.")
                + " PROPS OFF. It ends closed and then stops sending.")
        else:
            self.get_logger().error(
                f"Unknown command '{self.COMMAND}'. Use sweep, set, toggle "
                "or cycle.")
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
        if fn >= 1300:
            return (f"param5 {fn}  (PX4 function {fn - 1000} = "
                    f"Peripheral via Actuator Set {fn - 1300})")
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
                "/uav_1/fmu/out/vehicle_status`). Nothing here was refused, because "
                "nothing here was heard.")
            return

        if denied:
            self.get_logger().error(
                f"PX4 DENIED {len(denied)} of {len(seen)} outputs. " + DENIED_HELP)
            return

        if unsupported and len(unsupported) == len(seen) and self.encodings:
            # Right idea, wrong numbering scheme -- or the right numbering for
            # outputs this aircraft does not use. Walk the remaining encodings
            # rather than making someone read the MAVLink spec on a bench.
            self.FUNCTIONS = self.encodings.pop(0)
            self.step = 0
            self.results = {}
            self.get_logger().warning(
                "Every output came back UNSUPPORTED, which means the numbering "
                "was wrong rather than the servo. Retrying with the next "
                f"encoding: param5 {self.FUNCTIONS[0]}-{self.FUNCTIONS[-1]}"
                + (" -- the OFFBOARD ACTUATOR SET functions, which is where "
                   "this aircraft's release actually lives."
                   if self.FUNCTIONS is PX4_OFFBOARD_SET_FUNCTIONS else "."))
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

    def cycle_callback(self):
        """Walk a fixed list of held positions, one every HOLD_SECONDS.

        A LIST rather than a flip-flop with a counter, because the thing being
        proved is that the servo reaches BOTH ends N times and finishes at
        rest -- and an off-by-one in a flip-flop leaves it parked open, which
        on a loaded aircraft is the cone on the floor.
        """
        if self.cycle_done:
            return
        now = time.monotonic()

        if self.cycle_step < 0 or now >= self.cycle_next:
            self.cycle_step += 1
            if self.cycle_step >= len(self.cycle_steps):
                self.finish_cycle()
                return
            # The next boundary is the PREVIOUS boundary plus the hold, not
            # "now" plus the hold. Restarting from now adds however late this
            # 10 Hz tick was to every phase, and that error accumulates: a
            # measured run came out at 1.1 s per phase for a 1.0 s hold, which
            # over ten phases is a whole extra second. "Regular intervals"
            # should mean regular.
            self.cycle_next = (now if self.cycle_step == 0
                               else self.cycle_next) + self.HOLD_SECONDS
            label, value = self.cycle_steps[self.cycle_step]
            self.get_logger().warning(f"  {label}  ->  {value:+.2f}")

        _, value = self.cycle_steps[self.cycle_step]
        self.send_cycle_value(value)

    def finish_cycle(self):
        """Stop, having left the servo closed, and say what PX4 answered."""
        self.cycle_done = True
        self.timer.cancel()
        # One last closed command, so the final state is asserted rather than
        # merely being whatever the last hold happened to leave behind.
        self.send_cycle_value(self.CLOSED_VALUE)
        if self.acks == 0:
            self.get_logger().error(
                f"{self.CYCLES} cycles sent and NOT ONE ack from PX4. The "
                "commands are not arriving: check the uXRCE-DDS agent is "
                "connected (`ros2 topic hz /uav_1/fmu/out/vehicle_status`). Nothing "
                "was refused, because nothing was heard.")
        elif self.refusals:
            self.get_logger().error(
                f"Cycle finished, but PX4 REFUSED {self.refusals} of "
                f"{self.acks} commands. Last refusal: {self.last_refusal}. "
                + (DENIED_HELP if self.last_refusal == 'DENIED' else ""))
        else:
            self.get_logger().warning(
                f"Cycle finished: {self.CYCLES} full open/close cycles, "
                f"{self.acks} commands accepted by PX4, servo left CLOSED at "
                f"{self.CLOSED_VALUE:+.2f}. If PX4 accepted everything and the "
                "servo did not move, the output is not the one being "
                "addressed, or the servo rail is not powered -- a Pixhawk does "
                "not power it from the FMU, it needs BEC voltage on the rail.")
        self.get_logger().warning("Done. Ctrl-C to exit.")

    def send_cycle_value(self, value):
        if self.METHOD == 'set_actuator':
            self.send_do_set_actuator(value)
        else:
            self.testing = self.FUNCTION
            # A timeout slightly longer than the resend interval, so the
            # output is never briefly released between two commands, and well
            # under PX4's 3 s cap.
            self.send_actuator_test(self.FUNCTION, value, 1.0)

    def toggle_callback(self):
        if time.monotonic() - self.start_time > 2.0:
            self.position = (self.VALUE if self.position == self.NEUTRAL
                             else self.NEUTRAL)
            self.start_time = time.monotonic()
            self.get_logger().info(
                f"Offboard actuator set {self.INDEX} -> {self.position:+.2f}")

        self.send_do_set_actuator(self.position)

    def send_do_set_actuator(self, value):
        nan = float('nan')
        params = [nan] * 6          # NaN leaves the other outputs alone
        params[max(1, min(6, self.INDEX)) - 1] = float(value)
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
        if msg.result != 0:
            self.refusals += 1
            self.last_refusal = name
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
    if node.timer is None:
        # A usage error, already logged. Spinning on a node with nothing to do
        # leaves the operator staring at a process that will never say
        # anything else, which reads as a hang rather than as the mistake it
        # is. Exit and give the prompt back.
        node.destroy_node()
        rclpy.shutdown()
        return
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
