#!/usr/bin/env python3
"""
The course AND the thermal drop in one flight, joined by a handoff.

    [course_fsm, unchanged] window -> bars -> tubes -> last window
        -> HANDOFF  hold position and offer the aircraft to the thermal node
    [thermal_fsm, unchanged from here] HOLD -> (marker hunt ->) SURVEY climb
        -> APPROACH -> DESCEND -> HOVER/drop -> RETREAT -> LAND_* -> land

Two processes, one aircraft:

    mission_course   CourseFSM. Where the course would go to the pad
                     (_after_the_course, i.e. straight after the LAST window --
                     the second one with window_after_tubes, or the skip over
                     it) it enters HANDOFF instead: holds where it is and
                     publishes an offer on /mission/handoff at 20 Hz.
    mission_thermal  ThermalFSM, SILENT until an offer arrives -- no heartbeat,
                     no setpoint, no status. On a valid offer it takes the
                     course node's datum (home_z, arming point), hold point and
                     yaw, starts streaming the SAME hold setpoint, and acks.
                     Only on that ack does the course node stop streaming.

WHY NOT ONE CLASS
    CourseFSM is a WindowTraverse and ThermalFSM a ThermalDrop; they meet only
    at OffboardSequence, and both override the same dozen methods
    (timer_callback, _handle_hold, publish_status, the aruco callbacks...) and
    declare parameters with the same names meaning different things
    (takeoff_altitude, flight_seconds, pad_*). Merging them is a rewrite of
    both. Two processes keep both FSMs exactly as flown, each with its own
    launch file's parameters.

WHY THIS IS SAFE
    * The course node lets go ONLY on an ack. No thermal node, a thermal node
      in bench/dryrun, or one that thinks the vehicle is not armed/offboard/
      airborne never acks, and after handoff_timeout the course finishes the
      way it always did (pad or landing).
    * During the overlap both nodes publish the same hold, so the tick or two
      where both stream is harmless.
    * Same EKF, same local frame: home_z is carried over, so every altitude
      the thermal mission flies is still above the course's arming floor.

WHERE THE THERMAL MISSION STARTS (handoff_start on mission_thermal)
    survey  (default) hold hold_seconds where the course ended, then the
            survey: climb to survey_altitude right there and look for the
            boxes. Everything after is thermal_fsm as usual.
    marker  hold, then thermal_fsm's own marker hunt -> BOX_OFFSET -> survey.

    "Forward" for the thermal legs (the landing search/return) is the heading
    the aircraft settles on during that hold -- i.e. the direction it flew
    through the last window -- plus handoff_heading_offset_deg, unless
    leg_bearing_deg gives it outright.

    ros2 launch drone_testing course_thermal_mission.launch.py
"""

import json
import math
import time
import uuid

import numpy as np
import rclpy
from px4_msgs.msg import VehicleStatus
from std_msgs.msg import String

from drone_testing.course_fsm import CourseFSM
from drone_testing.offboard_sequence import spin_node, wrap_pi
from drone_testing.thermal_fsm import ThermalFSM

OFFER_TOPIC = '/mission/handoff'
ACK_TOPIC = '/mission/handoff_ack'


class MissionCourse(CourseFSM):

    HANDOFF = "HANDOFF"
    HANDOFF_TIMEOUT = 5.0       # s waiting for an ack before landing as usual

    def __init__(self):
        super().__init__()
        self.HANDOFF_ON = bool(self.declare_parameter('handoff', True).value)
        self.HANDOFF_TIMEOUT = float(self._declare_number(
            'handoff_timeout', self.HANDOFF_TIMEOUT))
        self.offer_pub = self.create_publisher(String, OFFER_TOPIC, 10)
        self.create_subscription(String, ACK_TOPIC, self._on_ack, 10,
                                 callback_group=self.sensor_cbg)
        self.handoff_token = None
        self.handoff_reason = None
        self.handoff_acked = False
        self.handed_off = False
        self.get_logger().warning(
            "MISSION: course first; after the last window the aircraft is "
            + ("handed to mission_thermal." if self.HANDOFF_ON else
               "NOT handed off (handoff:=false) -- plain course ending."))

    # ---- the hook -------------------------------------------------------

    def _after_the_course(self, reason):
        if not self.HANDOFF_ON:
            super()._after_the_course(reason)
            return
        self.handoff_reason = reason
        self.handoff_token = uuid.uuid4().hex[:12]
        self.handoff_acked = False
        self.moving = False
        self.yaw_remaining = 0.0
        self._restore_land_speed()
        self._enter_stage(self.HANDOFF)
        self.get_logger().warning(
            f"HANDOFF: course done ({reason}). Holding at "
            f"{self.relative_altitude() or 0.0:.2f} m and offering the "
            f"aircraft to the thermal mission (token {self.handoff_token}).")

    def _on_ack(self, msg):
        if self.handoff_token is not None and msg.data.strip() == self.handoff_token:
            self.handoff_acked = True

    def _offer(self):
        lp = self.local_position
        self.offer_pub.publish(String(data=json.dumps({
            'token': self.handoff_token,
            'stamp': time.time(),
            'home_x': self.home_x, 'home_y': self.home_y,
            'home_z': self.home_z,
            'heading': float(self._course_heading()),
            'yaw_setpoint': float(self.yaw_setpoint),
            'setpoint_z': float(self.setpoint_z if self.setpoint_z is not None else lp.z),
            'target_z': float(self.target_z if self.target_z is not None else lp.z),
            'commanded_altitude': float(self.commanded_altitude),
            'hold_xy': bool(self.hold_xy),
            'hold_x': None if self.hold_x is None else float(self.hold_x),
            'hold_y': None if self.hold_y is None else float(self.hold_y),
            'reason': self.handoff_reason,
        })))

    # ---- the stage ------------------------------------------------------

    def timer_callback(self):
        if self.current_stage != self.HANDOFF:
            super().timer_callback()
            return
        if self.stream_setpoints:
            self.publish_offboard_control_mode()
            self.publish_position_setpoint()
        self.publish_status()
        if self.kill_requested:
            self._enter_stage(self.KILLING)
            return
        if self.abort_requested:
            self.abort_requested = False
            self.handoff_token = None
            self._begin_landing("operator abort during handoff")
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()

        if self.handoff_acked:
            self.handed_off = True
            self.outcome = f"COURSE COMPLETE ({self.handoff_reason}); handed to thermal"
            self.get_logger().warning(
                "HANDOFF: acknowledged. The thermal mission is flying the "
                "aircraft; this node stops streaming now.")
            self._stand_down()
            return
        if self._in_stage_for() > self.HANDOFF_TIMEOUT:
            self.get_logger().error(
                f"HANDOFF: no ack in {self.HANDOFF_TIMEOUT:.0f} s -- is "
                "mission_thermal running, in mode fly, and seeing PX4? "
                "Finishing the course the normal way instead.")
            self.handoff_token = None
            self.HANDOFF_ON = False
            super()._after_the_course(self.handoff_reason)
            return
        self._offer()
        self.get_logger().info("HANDOFF: waiting for the thermal mission...",
                               throttle_duration_sec=1.0)

    def publish_status(self):
        if self.handed_off:
            return          # the status line belongs to the thermal node now
        if self.current_stage == self.HANDOFF:
            alt = self.relative_altitude()
            self.status_pub.publish(String(data="|".join([
                self.HANDOFF, 'ARM',
                f"{alt:.2f}" if alt is not None else 'nan',
                'POS' if self.hold_xy else '---', 'thermal?'])))
            return
        super().publish_status()


class MissionThermal(ThermalFSM):

    WAIT_HANDOFF = "WAIT_HANDOFF"
    MAX_OFFER_AGE = 2.0         # s; an older offer is not the aircraft now
    MIN_HANDOFF_ALT = 0.30      # m; below this it is not airborne enough

    def __init__(self):
        super().__init__()
        self.HANDOFF_START = str(self.declare_parameter(
            'handoff_start', 'survey').value).strip().lower()
        if self.HANDOFF_START not in ('survey', 'marker'):
            self.get_logger().error(
                f"handoff_start '{self.HANDOFF_START}' is not survey|marker; "
                "using survey.")
            self.HANDOFF_START = 'survey'
        self.HEADING_OFFSET = math.radians(float(self._declare_number(
            'handoff_heading_offset_deg', 0.0)))
        self.ack_pub = self.create_publisher(String, ACK_TOPIC, 10)
        self.create_subscription(String, OFFER_TOPIC, self._on_offer, 10,
                                 callback_group=self.sensor_cbg)
        self.dormant = self.MODE == 'fly'
        self.accepted_token = None
        self._offer_msg = None
        if self.dormant:
            self.current_stage = self.WAIT_HANDOFF
            self.get_logger().warning(
                "MISSION THERMAL: silent until the course hands the aircraft "
                f"over, then HOLD -> {self.HANDOFF_START} -> the thermal "
                "mission as usual.")

    def _keyboard_listener(self):
        return      # the course node owns the terminal

    def _on_offer(self, msg):
        self._offer_msg = msg.data

    def _refuse(self, why):
        self.get_logger().error(f"HANDOFF refused: {why}",
                                throttle_duration_sec=1.0)

    def _try_accept(self):
        raw, self._offer_msg = self._offer_msg, None
        if raw is None:
            return
        try:
            o = json.loads(raw)
        except ValueError:
            return self._refuse("unreadable offer")
        if o.get('token') == self.accepted_token:
            return
        if time.time() - float(o.get('stamp', 0.0)) > self.MAX_OFFER_AGE:
            return self._refuse("offer is stale")
        if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            return self._refuse("PX4 says the vehicle is not armed")
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return self._refuse("PX4 is not in Offboard")
        lp = self.local_position
        if lp is None or o.get('home_z') is None:
            return self._refuse("no local position / no datum yet")
        if o['home_z'] - lp.z < self.MIN_HANDOFF_ALT:
            return self._refuse("aircraft is not airborne")

        # Adopt the course node's frame and its hold, exactly as it is flying.
        self.home_x, self.home_y = o['home_x'], o['home_y']
        self.home_z = float(o['home_z'])
        self.home_yaw = wrap_pi(float(o['heading']) + self.HEADING_OFFSET)
        self.yaw_setpoint = float(o['yaw_setpoint'])
        self.setpoint_z = float(o['setpoint_z'])
        self.target_z = float(o['target_z'])
        self.commanded_altitude = float(o['commanded_altitude'])
        self.hold_xy = bool(o['hold_xy']) and o['hold_x'] is not None
        if self.hold_xy:
            self.hold_x, self.hold_y = float(o['hold_x']), float(o['hold_y'])
        self.moving = False
        self.yaw_remaining = 0.0
        self.blind_descent = False
        self.in_band_since = None
        self.stream_setpoints = True
        self.leg_samples.clear()
        self.flight_start = time.monotonic()
        self.accepted_token = o['token']
        self.dormant = False
        self._enter_stage(self.HOLD)
        self.ack_pub.publish(String(data=self.accepted_token))
        self.get_logger().warning(
            f"HANDOFF accepted ({o.get('reason')}): flying from "
            f"{self.relative_altitude() or 0.0:.2f} m, course heading "
            f"{math.degrees(float(o['heading'])):+.1f} deg. Holding "
            f"{self.HOLD_SECONDS:.1f} s, then {self.HANDOFF_START}.")

    def timer_callback(self):
        if self.dormant:
            self._try_accept()
            if self.dormant:
                self.get_logger().info("Waiting for the course handoff...",
                                       throttle_duration_sec=5.0)
                return
        elif self.accepted_token is not None:
            # Re-ack while the course is still offering (a lost ack).
            if self._offer_msg is not None and self.accepted_token in self._offer_msg:
                self.ack_pub.publish(String(data=self.accepted_token))
            self._offer_msg = None
        super().timer_callback()

    def _handle_hold(self):
        """Post-handoff hold: settle, learn the heading, then go."""
        if self.accepted_token is None:
            super()._handle_hold()
            return
        if not self._still_flyable():
            return
        self._try_latch_xy_hold()
        lp = self.local_position
        if lp is not None:
            self.leg_samples.append(float(lp.heading))
        if self._in_stage_for() < self.HOLD_SECONDS or not self.hold_xy:
            self.get_logger().info(
                "Holding after the handoff"
                + ('' if self.hold_xy else ' (waiting for flow x/y latch)') + "...",
                throttle_duration_sec=1.0)
            return
        self._latch_leg_yaw()
        if math.isnan(self.LEG_BEARING) and self.HEADING_OFFSET:
            self.leg_yaw = wrap_pi(self.leg_yaw + self.HEADING_OFFSET)
        if self.HANDOFF_START == 'marker':
            self._begin_mark_search()
        else:
            self._begin_survey(np.array([lp.x, lp.y]))

    def publish_status(self):
        if self.dormant:
            return
        super().publish_status()


def _run(cls, args):
    rclpy.init(args=args)
    node = None
    try:
        node = cls()
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_course(args=None):
    _run(MissionCourse, args)


def main_thermal(args=None):
    _run(MissionThermal, args)
