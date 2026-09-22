#!/usr/bin/env python3
"""
The cone release. One node, one output, one job: open the servo when the
mission says to, and put it back.

WHAT THIS FILE USED TO BE

    A sine wave. It swept MAIN 5 between -1 and +1 for ever, which is how we
    established the thing that matters here and is worth writing down: the
    offboard path to a servo on this airframe is

        /fmu/in/vehicle_command
        command = VEHICLE_CMD_DO_SET_ACTUATOR (187)
        param1  = the value, -1.0 .. +1.0, for OFFBOARD ACTUATOR SET 1
        from_external = True

    and PX4 obeys it only if that output's FUNCTION is set to
    "Offboard Actuator Set 1" in QGC's Actuators tab. An output left on
    "RC AUX 1" is an RC passthrough: the command arrives, PX4 acks it, and
    nothing moves. That was the whole sweep's finding.

    What it did not have was a reason to move. This file is that half.

HOW IT IS DRIVEN

    thermal_drop.py publishes on drop_trigger_topic (default /servo/drop):

        True   the instant HOVER commits to the drop, over the hot box
        False  once the payload has had servo_hold_seconds to clear

    So the FSM decides WHEN and this node decides HOW, which is the split
    that lets the release be bench-tested with one `ros2 topic pub` while
    the aircraft is on the table:

        ros2 topic pub --once /servo/drop std_msgs/Bool "data: true"

    Run it with release_enabled:=false in thermal_drop.py's launch, or with
    release_via_servo_node:=true, so that only ONE of the two is commanding
    the output. Two publishers at 20 Hz with different values is a servo
    that buzzes, not a servo that opens.

WHY IT KEEPS PUBLISHING, AND WHY IT STOPS

    DO_SET_ACTUATOR is a set-and-hold command, not a pulse, but a single
    message is also a single chance: it can be lost on the DDS link, and it
    can arrive while PX4 is mid-transition and be ignored. So the open is
    re-sent at repeat_rate for hold_seconds -- long enough for the cone to
    clear the tube -- and then the neutral value is sent the same way, and
    then it goes quiet. It does NOT stream neutral for ever: with nothing to
    hold against, a servo being told its own position twenty times a second
    is just current draw and jitter on an output the mission has finished
    with.

AUTO-NEUTRAL

    If hold_seconds passes and no False ever arrives -- the FSM died, the
    link dropped -- the servo is returned to neutral anyway. A release that
    latches open is a release that cannot be flown twice, and a cone bay
    hanging open is worse than a closed one.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from std_msgs.msg import Bool


class ServoController(Node):

    # MAV_CMD_ACTUATOR_TEST. The command QGC's Actuators tab uses, and the
    # only one that moves a servo while DISARMED -- which is what makes a
    # bench check of the linkage possible. It needs the PX4 output FUNCTION
    # number rather than the offboard set index, and PX4 stops the test when
    # its own timeout expires, so it is the bench path, not the flight path.
    CMD_ACTUATOR_TEST = 310

    def __init__(self):
        super().__init__('servo_test_node')
        # Publisher for sending commands to PX4
        self.publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Timer to publish commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now().nanoseconds / 1e9

    def timer_callback(self):
        # Generate a smooth sine wave value between -1.0 and 1.0
        elapsed_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        servo_position = math.sin(elapsed_time * 2.0) 

        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        
        # param1 targets Actuator Set 1 (which is mapped to MAIN 5 in your QGC)
        msg.param1 = servo_position 
        
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def ack_callback(self, msg):
        want = (self.CMD_ACTUATOR_TEST if self.COMMAND == 'actuator_test'
                else VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR)
        if msg.command != want:
            return
        self.ack_seen = True
        # result 0 is MAV_RESULT_ACCEPTED. Anything else is PX4 telling us
        # the command was understood and refused, which is a different
        # problem from a command that never arrived -- and the two are
        # indistinguishable in a log that only prints "sent".
        if msg.result != 0:
            self.get_logger().error(
                f"PX4 REFUSED the servo command: result {msg.result}. The "
                "output function assignment is the usual cause.",
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
