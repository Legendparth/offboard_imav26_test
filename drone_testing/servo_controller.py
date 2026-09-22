#!/usr/bin/env python3
"""The release node: one servo, one job.

This grew out of the sine-wave sweep that proved how to talk to the servo at
all -- MAV_CMD_DO_SET_ACTUATOR on /fmu/in/vehicle_command, with the output
assigned to "Offboard Actuator Set N" in QGC's Actuators tab.  That part is
unchanged and still the thing to fall back on when the wiring is in doubt.
What is new is the WHEN: instead of sweeping for ever, the node sits quiet
and waits for the flight node to publish True on drop_trigger_topic at the
instant the drop commits, opens the servo, holds it open long enough for the
cone to clear the bay, and then returns it to neutral.

It is a separate process from the flight node on purpose.  The release can be
run, watched and bench-tested by itself, with the aircraft on the table and
no mission in the air:

    ros2 run drone_testing servo_controller
    ros2 topic pub --once /servo/drop std_msgs/Bool "data: true"

Exactly one thing may drive the output.  When this node is running, the
flight node is launched with release_via_servo_node:=true and does not send
actuator commands at all -- two publishers sending different values to one
actuator at 20 Hz is a servo that buzzes rather than one that opens.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from std_msgs.msg import Bool


class ServoController(Node):

    def __init__(self):
        super().__init__('servo_controller')

        def p(name, default):
            return self.declare_parameter(name, default).value

        self.TOPIC = str(p('drop_trigger_topic', '/servo/drop'))
        self.DROP_VALUE = float(p('servo_drop_value', 1.0))
        self.NEUTRAL_VALUE = float(p('servo_neutral_value', -1.0))
        self.INDEX = int(p('servo_index', 1))
        self.HOLD_SECONDS = float(p('servo_hold_seconds', 2.0))
        # PX4 acts on the LAST command it received, so a single packet that is
        # dropped is a cone that never leaves.  Re-send while holding open.
        self.REPEAT_RATE = float(p('repeat_rate', 10.0))
        self.COMMAND = str(p('servo_command', 'set_actuator')).lower()
        self.FUNCTION = int(p('servo_function', 0))
        self.CLOSE_ON_START = bool(p('close_on_start', False))

        if self.REPEAT_RATE <= 0.0:
            self.REPEAT_RATE = 10.0
        if self.HOLD_SECONDS <= 0.0:
            self.HOLD_SECONDS = 0.5

        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # PX4's /fmu/out/... topics are published BEST_EFFORT.  A subscriber
        # that asks for RELIABLE is silently never delivered anything, and
        # this node would then report "PX4 never acked" on every single drop.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.ack_callback, px4_qos)
        self.create_subscription(Bool, self.TOPIC, self.trigger_callback, 10)

        self.open_until = None      # monotonic deadline, or None when closed
        self.drops = 0
        self.awaiting_ack = False
        self.acked = False

        self.timer = self.create_timer(1.0 / self.REPEAT_RATE, self.tick)

        if self.CLOSE_ON_START:
            # Only on request.  On a loaded, armed vehicle an unasked-for
            # servo command is not a courtesy.
            self.send(self.NEUTRAL_VALUE)
            self.get_logger().info(
                f"Bay set known-closed: servo to {self.NEUTRAL_VALUE:+.2f}.")

        how = (f"MAV_CMD_ACTUATOR_TEST, function {self.FUNCTION}"
               if self.COMMAND == 'actuator_test'
               else f"MAV_CMD_DO_SET_ACTUATOR, offboard set {self.INDEX}")
        self.get_logger().info(
            f"Servo release ready on {self.TOPIC}. "
            f"open={self.DROP_VALUE:+.2f} neutral={self.NEUTRAL_VALUE:+.2f} "
            f"hold={self.HOLD_SECONDS:.1f}s via {how}.")

    # ---------------------------------------------------------------- trigger

    def trigger_callback(self, msg):
        if msg.data:
            now = time.monotonic()
            if self.open_until is not None:
                # A re-trigger extends the hold, it does not stack a second
                # one behind the first.
                self.open_until = now + self.HOLD_SECONDS
                self.get_logger().info("Drop re-triggered: hold extended.")
                return
            self.drops += 1
            self.open_until = now + self.HOLD_SECONDS
            self.awaiting_ack = True
            self.acked = False
            self.send(self.DROP_VALUE, force=True)
            self.get_logger().warning(
                f"DROP #{self.drops}: servo to {self.DROP_VALUE:+.2f}, "
                f"holding {self.HOLD_SECONDS:.1f} s.")
        else:
            # The flight node saying the payload has cleared.  If it never
            # says so, tick() closes on the deadline anyway.
            if self.open_until is not None:
                self.close("flight node released the hold")

    # ------------------------------------------------------------------- loop

    def tick(self):
        if self.open_until is None:
            return
        if time.monotonic() >= self.open_until:
            self.close(f"held {self.HOLD_SECONDS:.1f} s")
            return
        self.send(self.DROP_VALUE)

    def close(self, why):
        self.open_until = None
        self.send(self.NEUTRAL_VALUE, force=True)
        self.get_logger().info(
            f"Servo back to neutral {self.NEUTRAL_VALUE:+.2f} ({why}).")
        if self.awaiting_ack and not self.acked:
            # Say the right thing for the command actually sent: the two have
            # different failure causes and telling someone to check the
            # offboard actuator set when they are using ACTUATOR_TEST, which
            # does not use it, is a wasted bench session.
            if self.COMMAND == 'actuator_test':
                why = ("servo_function is 0, i.e. unset. Find the real one "
                       "with `ros2 run drone_testing servo_test --ros-args "
                       "-p command:=sweep`."
                       if self.FUNCTION == 0 else
                       f"PX4 refused servo_function {self.FUNCTION}. DENIED "
                       "means armed, or a safety button not pressed, or "
                       "COM_MOT_TEST_EN != 1. UNSUPPORTED means the number "
                       "is in the wrong encoding: param5 wants 33-40 for "
                       "Servo 1-8, or 1201-1208, not the tab's \"Servo 4\". "
                       "The sweep prints the number to use.")
            else:
                why = (f"the output must be assigned to \"Offboard Actuator "
                       f"Set {self.INDEX}\" in QGC, and the vehicle must be "
                       "ARMED -- PX4 ignores DO_SET_ACTUATOR while disarmed. "
                       "For a disarmed bench test use "
                       "servo_command:=actuator_test with servo_function set.")
            self.get_logger().error(
                "PX4 never acked the actuator command. Either nothing is "
                f"carrying it (check the uXRCE-DDS agent) or {why}")
        self.awaiting_ack = False

    # -------------------------------------------------------------- px4 wires

    def send(self, value, force=False):
        nan = float('nan')
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.COMMAND == 'actuator_test':
            # What QGC's Actuators tab uses.  This is the one that works while
            # DISARMED, so it is the only way a bench test moves the servo --
            # but it wants the PX4 output FUNCTION number, not the offboard
            # set index, and PX4 stops the test when its timeout expires.
            msg.command = 310                   # MAV_CMD_ACTUATOR_TEST
            msg.param1 = float(value)
            msg.param2 = float(self.HOLD_SECONDS)
            msg.param3 = msg.param4 = 0.0
            msg.param5 = float(self.FUNCTION)
            msg.param6 = msg.param7 = 0.0
        else:
            params = [nan] * 6                  # NaN leaves an output alone
            params[max(1, min(6, self.INDEX)) - 1] = float(value)
            msg.command = 187                   # MAV_CMD_DO_SET_ACTUATOR
            (msg.param1, msg.param2, msg.param3,
             msg.param4, msg.param5, msg.param6) = params
            msg.param7 = 0.0                    # actuator set index group
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def ack_callback(self, msg):
        if msg.command not in (187, 310):
            return
        if msg.result == 0:
            if not self.acked:
                self.acked = True
                self.get_logger().info("PX4 accepted the actuator command.")
        else:
            self.acked = True       # it answered; it just said no
            self.get_logger().error(
                f"PX4 REFUSED the actuator command (result {msg.result}). "
                "The output is most likely not assigned to an offboard "
                "actuator set.")


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.open_until is not None:
            node.send(node.NEUTRAL_VALUE, force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
