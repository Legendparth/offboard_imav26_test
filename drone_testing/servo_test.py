#!/usr/bin/env python3
"""Find the servo, and prove it moves, before any of the rest of it.

The mission is a slow way to debug a servo. This node talks to one directly
and tells you which PX4 output it is on, which is the number everything else
then needs.

    command:=sweep      (the default) MAV_CMD_ACTUATOR_TEST across Servo 1-8,
                        which are output FUNCTIONS 201-208, one per second,
                        printing PX4's ack for each. WATCH THE SERVO. The
                        function that is being tested when it moves is the
                        number to give servo_function everywhere else.

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

# PX4 output functions. Servo 1-8 are 201-208; these are what a payload servo
# is realistically wired to.
SERVO_FUNCTIONS = list(range(201, 209))


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
        self.FUNCTIONS = [int(f) for f in p('functions', SERVO_FUNCTIONS)]

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
            self.get_logger().warning(
                f"Sweep finished. {self.acks} ack(s) from PX4. If NOTHING "
                "moved: the servo may not be on Servo 1-8, or the FMU is not "
                "powering the rail, or the uXRCE-DDS agent is not connected "
                "(no acks at all means nothing arrived). If something moved, "
                "use that function number for servo_function.")
            self.timer.cancel()
            return
        fn = self.FUNCTIONS[self.step]
        self.step += 1
        self.testing = fn
        # param2 is the test timeout: PX4 returns the output to its default
        # when it expires, so each step tidies up after itself.
        self.send_actuator_test(fn, self.VALUE, self.STEP_SECONDS)
        self.get_logger().info(
            f"  function {fn}  (Servo {fn - 200})  ->  {self.VALUE:+.2f}")

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
        where = (f"function {self.testing}" if self.testing
                 else f"actuator set {self.INDEX}")
        if msg.result == 0:
            self.get_logger().info(f"    PX4 accepted {where}.",
                                   throttle_duration_sec=1.0)
        else:
            self.get_logger().error(
                f"    PX4 REFUSED {where}, result {msg.result}.",
                throttle_duration_sec=1.0)


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
