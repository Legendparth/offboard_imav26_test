#!/usr/bin/env python3
"""
Move the drop servo from ROS, and say what PX4 thought of it.

The drop in thermal_drop did not move the servo, and the mission log can only
ever say "command sent". This node sends the same commands and also listens to
/fmu/out/vehicle_command_ack, so you get PX4's own verdict:

    ACCEPTED              PX4 ran it. If nothing moved, the OUTPUT MAPPING is
                          wrong -- the command reached a function no servo is
                          assigned to.
    DENIED / TEMPORARILY_REJECTED
                          PX4 refused. Disarmed refusal of DO_SET_ACTUATOR is
                          the expected case: offboard actuator values are
                          applied to the outputs only while ARMED, and a
                          disarmed output sits at its Disarmed value instead.
    UNSUPPORTED           This firmware does not implement that command.
    (no ack at all)       The command never reached PX4 -- agent down, wrong
                          topic, or DDS not carrying vehicle_command.

TWO COMMANDS, AND WHEN EACH ONE WORKS

    command:=set_actuator    MAV_CMD_DO_SET_ACTUATOR. What the mission uses in
                             flight. Needs the output assigned to "Offboard
                             Actuator Set <index>" in QGC's Actuators tab.
                             Expect it to do nothing while disarmed.

    command:=actuator_test   MAV_CMD_ACTUATOR_TEST -- what the QGC Actuators
                             tab sliders send, and the one that works on the
                             bench with the vehicle DISARMED. It addresses the
                             output by PX4 FUNCTION NUMBER, so set function:=.

    command:=sweep           Tries actuator_test across a range of function
                             numbers, printing each and alternating open/closed
                             on it. Watch the servo and note which number moves
                             it. Defaults to the whole servo space: 201-208
                             (Servo 1-8) and 301-306 (Peripheral via Actuator
                             Set 1-6). MOTOR functions (101-112) are excluded
                             on purpose -- sweeping one would spin a prop.

USE

    ros2 run drone_testing servo_test --ros-args -p command:=actuator_test \\
        -p function:=201

    ros2 run drone_testing servo_test --ros-args -p command:=sweep

Nothing here ever arms the vehicle or publishes a setpoint. PROPS OFF anyway:
you are commanding actuators on a real airframe.
"""

import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DO_SET_ACTUATOR = 187
ACTUATOR_TEST = 310

RESULTS = {
    0: 'ACCEPTED -- PX4 executed it. If the servo did not move, the output '
       'mapping is wrong, not the command.',
    1: 'TEMPORARILY_REJECTED -- PX4 will not do it right now (disarmed is the '
       'usual reason).',
    2: 'DENIED -- PX4 refuses it in this state.',
    3: 'UNSUPPORTED -- this firmware does not implement that command.',
    4: 'FAILED -- PX4 tried and it failed.',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


class ServoTest(Node):

    def __init__(self):
        super().__init__('servo_test')

        self.command = str(self.declare_parameter('command', 'actuator_test')
                           .value).strip().lower()
        if self.command not in ('set_actuator', 'actuator_test', 'sweep'):
            self.get_logger().error(
                f"command '{self.command}' is not set_actuator | actuator_test "
                "| sweep; using actuator_test.")
            self.command = 'actuator_test'

        self.index = int(self.declare_parameter('index', 1).value)
        self.function = int(self.declare_parameter('function', 201).value)
        self.open_value = float(self.declare_parameter('open_value', 1.0).value)
        self.closed_value = float(self.declare_parameter('closed_value', -1.0).value)
        self.hold = float(self.declare_parameter('hold_seconds', 2.0).value)
        self.cycles = int(self.declare_parameter('cycles', 3).value)
        self.sweep_from = int(self.declare_parameter('sweep_from', 201).value)
        self.sweep_to = int(self.declare_parameter('sweep_to', 312).value)

        # 201-208 are Servo 1-8; the 300 block is Peripheral via Actuator Set.
        # 101-112 are the MOTORS, and this node refuses to address them: a
        # swept motor function spins a propeller.
        # 300-312 rather than 301-306: the exact base of the Actuator Set
        # block is what we are trying to pin down, so cover an off-by-one at
        # either end. Nothing in 300-312 is a motor, and the 400 block (landing
        # gear, parachute) is deliberately left out.
        self.sweep_list = [fn for fn in range(self.sweep_from, self.sweep_to + 1)
                           if 201 <= fn <= 208 or 300 <= fn <= 312]
        if not self.sweep_list:
            self.get_logger().error(
                f"sweep range {self.sweep_from}-{self.sweep_to} contains no "
                "servo functions. Valid bands are 201-208 (Servo 1-8) and "
                "300-312 (the Peripheral via Actuator Set block). Motor "
                "functions are 101-112 and will not be swept. Sweeping "
                "201-208 and 300-312 instead.")
            self.sweep_list = list(range(201, 209)) + list(range(300, 313))

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        ack_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.ack_callback, ack_qos)

        self.acks = 0
        self.sent = 0
        self.step = 0
        self.next_at = time.monotonic() + 1.0
        self.create_timer(0.1, self.tick)

        if self.command == 'sweep':
            self.get_logger().warning(
                "SWEEP: actuator_test on functions "
                + ', '.join(str(fn) for fn in self.sweep_list)
                + f", {self.hold:.1f} s each, alternating "
                f"{self.open_value:+.2f} / {self.closed_value:+.2f} so the "
                "movement is unmistakable. WATCH THE SERVO and note which "
                "function number moves it. Props off.")
        else:
            self.get_logger().warning(
                f"{self.command}: {self.cycles} cycles of "
                f"{self.open_value:+.2f} -> {self.closed_value:+.2f}, "
                f"{self.hold:.1f} s apart, on "
                + (f"function {self.function}." if self.command == 'actuator_test'
                   else f"offboard actuator set {self.index}.")
                + " Props off.")

    # ------------------------------------------------------------------ acks

    def ack_callback(self, msg):
        if msg.command not in (DO_SET_ACTUATOR, ACTUATOR_TEST):
            return
        self.acks += 1
        name = ('DO_SET_ACTUATOR' if msg.command == DO_SET_ACTUATOR
                else 'ACTUATOR_TEST')
        self.get_logger().warning(
            f"PX4 ack for {name}: "
            f"{RESULTS.get(msg.result, f'result {msg.result}')}")

    # -------------------------------------------------------------- commands

    def send(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        for i in range(1, 8):
            setattr(msg, f'param{i}', float(params.get(f'param{i}', 0.0)))
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        self.sent += 1

    def set_actuator(self, value):
        nan = float('nan')
        params = {f'param{i}': nan for i in range(1, 7)}
        params[f'param{max(1, min(6, self.index))}'] = value
        params['param7'] = 0.0
        self.send(DO_SET_ACTUATOR, **params)

    @staticmethod
    def _name(fn):
        # The base of the Actuator Set block is unconfirmed, so report the
        # raw number and both candidate readings rather than assert one.
        return (f"Servo {fn - 200}" if fn < 300
                else f"Actuator Set {fn - 300} if 0-based, {fn - 300 + 1} if 1-based")

    def actuator_test(self, value, function):
        # param1 value, param2 timeout, param5 output function.
        self.send(ACTUATOR_TEST, param1=value, param2=self.hold,
                  param5=float(function))

    # ----------------------------------------------------------------- drive

    def tick(self):
        now = time.monotonic()
        if now < self.next_at:
            return
        self.next_at = now + self.hold

        if self.command == 'sweep':
            # Two slots per function: open then closed, so a servo already
            # parked at the open position still visibly moves.
            if self.step >= len(self.sweep_list) * 2:
                self._finish()
                return
            fn = self.sweep_list[self.step // 2]
            value = self.open_value if self.step % 2 == 0 else self.closed_value
            self.get_logger().warning(
                f"--> function {fn} ({self._name(fn)}): {value:+.2f}")
            self.actuator_test(value, fn)
            self.step += 1
            return

        if self.step >= self.cycles * 2:
            self._finish()
            return
        value = self.open_value if self.step % 2 == 0 else self.closed_value
        self.get_logger().warning(f"--> {value:+.2f}")
        if self.command == 'actuator_test':
            self.actuator_test(value, self.function)
        else:
            self.set_actuator(value)
        self.step += 1

    def _finish(self):
        if self.acks == 0:
            self.get_logger().error(
                f"Sent {self.sent} commands and PX4 acknowledged NONE of them. "
                "Either the uXRCE-DDS agent is not running, or "
                "vehicle_command_ack is not in the PX4 DDS topic list. Check "
                "`ros2 topic hz /fmu/out/vehicle_status_v1` first -- if that "
                "is silent, nothing is reaching PX4 at all.")
        else:
            self.get_logger().warning(
                f"Done: {self.sent} commands, {self.acks} acks. If they were "
                "ACCEPTED and the servo still did not move, the output is not "
                "assigned to the function that was commanded -- fix it in "
                "QGC -> Actuators.")
        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoTest()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
