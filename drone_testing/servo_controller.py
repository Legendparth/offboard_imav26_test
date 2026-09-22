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
        super().__init__('servo_controller')

        self.TRIGGER_TOPIC = str(self.declare_parameter(
            'drop_trigger_topic', '/servo/drop').value)
        # -1.0 .. +1.0, the raw offboard actuator value. Which end of that
        # range is "open" is a property of the horn, the linkage and the
        # servo's direction, so BOTH ends are parameters and neither has a
        # sensible universal default. Set them on the bench, with the cone
        # in, by publishing on the trigger topic and watching the hatch.
        self.OPEN_VALUE = float(self.declare_parameter('servo_drop_value', 1.0).value)
        self.NEUTRAL_VALUE = float(self.declare_parameter(
            'servo_neutral_value', -1.0).value)
        self.SERVO_INDEX = int(self.declare_parameter('servo_index', 1).value)
        self.HOLD_SECONDS = float(self.declare_parameter(
            'servo_hold_seconds', 2.0).value)
        self.REPEAT_RATE = float(self.declare_parameter('repeat_rate', 10.0).value)
        self.COMMAND = str(self.declare_parameter(
            'servo_command', 'set_actuator').value).strip().lower()
        if self.COMMAND not in ('set_actuator', 'actuator_test'):
            self.get_logger().error(
                f"servo_command '{self.COMMAND}' is not set_actuator or "
                "actuator_test; using set_actuator.")
            self.COMMAND = 'set_actuator'
        self.SERVO_FUNCTION = int(self.declare_parameter('servo_function', 0).value)
        if self.COMMAND == 'actuator_test' and self.SERVO_FUNCTION <= 0:
            self.get_logger().error(
                "servo_command is actuator_test but servo_function is 0. That "
                "is the PX4 OUTPUT FUNCTION number of the servo, the same one "
                "QGC's Actuators tab shows against that output -- not the "
                "offboard set index. Nothing will move until it is set.")
        # true = go to neutral at startup, so the bay is known-closed before
        # the aircraft ever leaves the ground. Off by default: on a vehicle
        # that is already loaded and armed, an unasked-for servo command is
        # not a courtesy.
        self.CLOSE_ON_START = bool(self.declare_parameter(
            'close_on_start', False).value)

        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.create_subscription(Bool, self.TRIGGER_TOPIC,
                                 self.trigger_callback, 10)
        # PX4's /fmu/out/... topics come off the DDS bridge BEST_EFFORT, and a
        # RELIABLE subscriber does not match one: rclpy says so and then
        # receives nothing at all --
        #     "offering incompatible QoS. No messages will be received from
        #      it. Last incompatible policy: RELIABILITY"
        # -- which would leave this node reporting "PX4 never acked" on every
        # single drop, the one message that is supposed to mean something.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleCommandAck, '/fmu/out/vehicle_command_ack',
                                 self.ack_callback, px4_qos)

        # What the output is being held at, and until when. None = quiet.
        self.value = None
        self.until = 0.0
        self.opened_at = None
        self.ack_seen = False
        self.drops = 0

        self.timer = self.create_timer(1.0 / max(1.0, self.REPEAT_RATE),
                                       self.timer_callback)

        self.get_logger().warning(
            f"SERVO CONTROLLER ready. Waiting for True on {self.TRIGGER_TOPIC}; "
            f"open = {self.OPEN_VALUE:+.2f}, neutral = {self.NEUTRAL_VALUE:+.2f} "
            + (f"on offboard actuator set {self.SERVO_INDEX}"
               if self.COMMAND == 'set_actuator'
               else f"via actuator_test function {self.SERVO_FUNCTION}")
            + f", held {self.HOLD_SECONDS:.1f} s at {self.REPEAT_RATE:.0f} Hz. "
            "The PX4 output must be assigned to that function in QGC, or "
            "nothing moves however many commands arrive.")
        if self.CLOSE_ON_START:
            self._hold(self.NEUTRAL_VALUE, 0.5)
            self.get_logger().info("close_on_start: sending neutral once.")

    # --------------------------------------------------------------- driving

    def _hold(self, value, seconds):
        """Command a value, and keep commanding it for `seconds`."""
        self.value = float(value)
        self.until = time.monotonic() + float(seconds)
        self._send(self.value)

    def trigger_callback(self, msg):
        if msg.data:
            if self.opened_at is not None:
                # Already open. Re-arm the hold rather than stack two of
                # them, so a repeated trigger extends the opening instead of
                # closing it early.
                self.opened_at = time.monotonic()
                self._hold(self.OPEN_VALUE, self.HOLD_SECONDS)
                self.get_logger().warning("DROP re-triggered; hold restarted.")
                return
            self.drops += 1
            self.opened_at = time.monotonic()
            self.ack_seen = False
            self._hold(self.OPEN_VALUE, self.HOLD_SECONDS)
            self.get_logger().warning(
                f"DROP #{self.drops}: servo to {self.OPEN_VALUE:+.2f}, held "
                f"{self.HOLD_SECONDS:.1f} s.")
        else:
            if self.opened_at is None:
                return
            self._close("the mission said the payload is clear")

    def _close(self, why):
        self.opened_at = None
        # Half a second of neutral, then silence. Long enough that the
        # message cannot simply have been lost; short enough that the output
        # is not being driven for the rest of the flight.
        self._hold(self.NEUTRAL_VALUE, 0.5)
        self.get_logger().warning(
            f"Servo back to neutral ({self.NEUTRAL_VALUE:+.2f}): {why}."
            + ("" if self.ack_seen else
               " PX4 never acked the open command -- if the cone did not go, "
               "the command never arrived: check the DDS agent."))

    def timer_callback(self):
        now = time.monotonic()
        if self.opened_at is not None and now - self.opened_at > self.HOLD_SECONDS:
            self._close("hold expired with no all-clear from the mission")
            return
        if self.value is None or now > self.until:
            self.value = None
            return
        self._send(self.value)

    # ---------------------------------------------------------------- the wire

    def _send(self, value):
        nan = float('nan')
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        if self.COMMAND == 'actuator_test':
            msg.command = self.CMD_ACTUATOR_TEST
            msg.param1 = float(value)
            msg.param2 = float(self.HOLD_SECONDS)
            msg.param3 = msg.param4 = 0.0
            msg.param5 = float(self.SERVO_FUNCTION)
            msg.param6 = msg.param7 = 0.0
        else:
            # NaN on every other slot: DO_SET_ACTUATOR sets the whole set in
            # one message, and a 0.0 in an unused slot would centre a servo
            # this node has no business touching.
            params = [nan] * 6
            params[max(1, min(6, self.SERVO_INDEX)) - 1] = float(value)
            msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
            (msg.param1, msg.param2, msg.param3,
             msg.param4, msg.param5, msg.param6) = params
            msg.param7 = 0.0       # actuator set index group
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
