#!/usr/bin/env python3
"""
Hold the MG90S at one commanded position, from ROS, in offboard flight.

THE OUTPUT IS "PERIPHERAL VIA ACTUATOR SET 1", SO THE FUNCTION IS 301

    PX4 addresses outputs by FUNCTION NUMBER, not by connector pin:

        101-112   Motor 1-12
        201-208   Servo 1-8
        301-306   Peripheral via Actuator Set 1-6   <-- ours is 301

    The old version of this file sent function 201 (Servo 1). Nothing is
    assigned to Servo 1, so PX4 accepted the command and moved nothing. With
    the output set to "Peripheral via Actuator Set 1" the number is 301.

TWO WAYS TO DRIVE IT, AND WHEN EACH ONE APPLIES

    mode:=actuator_test   MAV_CMD_ACTUATOR_TEST (310). Exactly what the QGC
                          Actuators slider sends. Addresses function 301
                          directly and works DISARMED, so this is the bench
                          test. It carries a timeout, so it must be resent
                          faster than the timeout or the output snaps back to
                          its Disarmed value.

    mode:=servos          Publishes ActuatorServos on /fmu/in/actuator_servos
                          at 50 Hz -- no vehicle command at all. This is the
                          fallback for when the command path acks but nothing
                          moves. It feeds functions 201-208, NOT 301, so it
                          only reaches a pin if you reassign the output to
                          "Servo 1" in QGC. Armed only, like set_actuator.

    mode:=set_actuator    MAV_CMD_DO_SET_ACTUATOR (187). The one for FLIGHT.
                          paramN is the value for Actuator Set N, so Set 1 is
                          param1 and the rest are NaN ("leave alone"). PX4
                          applies it only while ARMED -- disarmed it will ack
                          fine and do nothing. It does NOT need offboard mode
                          or OffboardControlMode flags; it is independent of
                          the setpoint stream, so it works in the middle of an
                          offboard mission without disturbing it.

VALUE -> ANGLE

    PX4 maps value -1.0 .. +1.0 onto that output's PWM_*_MIN .. PWM_*_MAX.
    With the usual MIN=1000 / MAX=2000 and an MG90S (1000us=0deg,
    1500us=90deg, 2000us=180deg):

        value -1.0  ->  1000us  ->    0 deg
        value  0.0  ->  1500us  ->   90 deg   <-- the default here
        value +1.0  ->  2000us  ->  180 deg

    So 90 degrees is value 0.0, and the node defaults to it. If you also want
    it to sit at 90 while disarmed, set that output's Disarmed value to 1500
    in QGC -> Actuators (PWM_MAIN_DISn / PWM_AUX_DISn), not 1000.

USE

    bench, disarmed:
        ros2 run drone_testing servo_control --ros-args \
            -p mode:=actuator_test -p value:=0.0

    in offboard flight (armed):
        ros2 run drone_testing servo_control --ros-args \
            -p mode:=set_actuator -p value:=0.0

PROPS OFF on the bench. This node never arms and never publishes a setpoint.
"""

import rclpy
from px4_msgs.msg import ActuatorServos, VehicleCommand, VehicleCommandAck
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

DO_SET_ACTUATOR = 187
ACTUATOR_TEST = 310

RESULTS = {
    0: 'ACCEPTED -- PX4 ran it. If nothing moved, the function number is wrong '
       'for this output.',
    1: 'TEMPORARILY_REJECTED -- not right now (disarmed is the usual reason).',
    2: 'DENIED',
    3: 'UNSUPPORTED -- this firmware does not implement the command.',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


class ServoControl(Node):

    def __init__(self):
        super().__init__('servo_control')

        self.mode = str(self.declare_parameter('mode', 'actuator_test')
                        .value).strip().lower()
        if self.mode not in ('actuator_test', 'set_actuator', 'servos'):
            self.get_logger().error(
                f"mode '{self.mode}' is not actuator_test | set_actuator | "
                "servos; using actuator_test.")
            self.mode = 'actuator_test'

        # 0.0 = mid travel = 90 deg on an MG90S with MIN/MAX at 1000/2000.
        self.value = float(self.declare_parameter('value', 0.0).value)
        self.value = max(-1.0, min(1.0, self.value))
        # 301 = Peripheral via Actuator Set 1 (actuator_test addresses this).
        self.function = int(self.declare_parameter('function', 301).value)
        # Actuator Set 1 -> param1 (set_actuator addresses this).
        self.index = max(1, min(6, int(self.declare_parameter('index', 1).value)))
        # The actuator_test timeout. Resent at half of it so it never lapses.
        self.timeout = float(self.declare_parameter('timeout', 1.0).value)

        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        # Only used by mode:=servos. PX4 wants this at a steady rate, and the
        # FunctionServos provider reads it for functions 201-208, so the output
        # must be reassigned to "Servo 1" for this mode to reach a pin.
        self.servo_pub = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', 10)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.on_ack,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=5))

        self.acks = 0
        # servos wants a steady 50 Hz stream; the command modes only need to
        # beat the actuator_test timeout.
        period = 0.02 if self.mode == 'servos' else max(0.1, self.timeout / 2.0)
        self.create_timer(period, self.tick)

        if self.mode == 'actuator_test':
            where = f"function {self.function}"
        elif self.mode == 'set_actuator':
            where = f"actuator set {self.index}"
        else:
            where = f"actuator_servos control[{self.index - 1}] (Servo {self.index})"

        self.get_logger().warning(
            f"{self.mode}: holding {where} at {self.value:+.2f} "
            f"({1500 + self.value * 500:.0f} us with MIN/MAX 1000/2000, "
            f"~{90 + self.value * 90:.0f} deg on an MG90S). Ctrl-C to stop.")

    def on_ack(self, msg):
        if msg.command not in (DO_SET_ACTUATOR, ACTUATOR_TEST):
            return
        self.acks += 1
        if self.acks > 1:
            return  # once is enough; this repeats several times a second
        name = ('DO_SET_ACTUATOR' if msg.command == DO_SET_ACTUATOR
                else 'ACTUATOR_TEST')
        self.get_logger().warning(
            f"PX4 ack for {name}: "
            f"{RESULTS.get(msg.result, f'result {msg.result}')}")

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

    def tick(self):
        if self.mode == 'servos':
            msg = ActuatorServos()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.timestamp_sample = msg.timestamp
            # NaN means "disarmed" to PX4, so every unused channel stays NaN
            # and only ours carries a value.
            msg.control = [float('nan')] * 8
            msg.control[self.index - 1] = self.value
            self.servo_pub.publish(msg)
            return

        if self.mode == 'actuator_test':
            # param1 value, param2 timeout, param5 output function.
            self.send(ACTUATOR_TEST, param1=self.value, param2=self.timeout,
                      param5=float(self.function))
        else:
            # NaN on the sets we are not touching.
            nan = float('nan')
            params = {f'param{i}': nan for i in range(1, 7)}
            params[f'param{self.index}'] = self.value
            params['param7'] = 0.0
            self.send(DO_SET_ACTUATOR, **params)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.acks == 0 and node.mode != 'servos':
                node.get_logger().error(
                    "PX4 acknowledged NOTHING. Check the uXRCE-DDS agent is up: "
                    "`ros2 topic hz /fmu/out/vehicle_status_v1`.")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
