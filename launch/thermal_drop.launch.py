"""
Thermal drop: find the hottest of three boxes with the MLX90640 and hover the
drop point over it at drop height. ARK Flow + Pixhawk IMU localisation.

    uXRCE-DDS agent + thermal_sensor (MLX90640 over I2C) + thermal_drop (flight)

WHAT TO RUN, IN THIS ORDER

1. Bench, props off, nothing sent to PX4. Hold something hot under the
   camera, move it to the drone's RIGHT and FORWARD and check the log agrees:

       ros2 launch drone_testing thermal_drop.launch.py mode:=bench agent_only:=false

   If RIGHT/LEFT or FORWARD/BACK is inverted, fix flip_lr / flip_ud /
   cam_yaw_deg before flying.

2. Flight with q/k keyboard aborts (a launched node has no tty):

       ros2 launch drone_testing thermal_drop.launch.py
       ros2 run drone_testing thermal_drop --ros-args \\
           -p cam_from_drop_forward:=0.10 -p box_height:=0.25

3. DRY RUN -- the whole algorithm, LED and servo, with the motors dead.
   Props off. Nothing is armed and not one setpoint is published; the
   simulated vehicle starts at survey_altitude and you walk it down by
   centring a hot object on the crosshair at http://<jetson>:8082/ :

       ros2 launch drone_testing thermal_drop.launch.py mode:=dryrun agent_only:=false

   Servo wiring check on its own, before that:

       ros2 launch drone_testing thermal_drop.launch.py mode:=dryrun \\
           agent_only:=false servo_test_on_start:=true

4. Everything in one shot (RC kill switch still works, keyboard does not):

       ros2 launch drone_testing thermal_drop.launch.py agent_only:=false

THE WHOLE MISSION, NOT JUST THE DROP -- AND IT IS THE DEFAULT

    fsm defaults to TRUE, so the flight node is thermal_fsm and step 4
    above flies the whole competition run:

        takeoff to 2.50 m -> forward up to 9.00 m to an ArUco marker
        -> hover 1 s on it -> 2.20 m RIGHT onto the boxes -> watch all
        three from 2.50 m and take the hottest -> down to 0.50 m over it
        -> drop -> climb and step 2.20 m RIGHT onto the corridor
        -> square up on the marker there (a DATUM, not the pad)
        -> BACKWARDS up to 9.00 m down the corridor to the LANDING marker
        -> precision landing on it

    THE THREE MEASURED DISTANCES, ALL CAPS RATHER THAN LEGS

        mark_search_distance    9.00 m   takeoff marker -> the next one
        land_return_distance    9.00 m   datum marker   -> the landing pad
        box_offset_right        2.20 m   marker -> the centre of the boxes
        retreat_right           2.20 m   the boxes -> the datum's corridor

        The two 9 m numbers are the SAME 8.7-8.8 m gap measured in the real
        arena, plus slack, because the way out and the way home are the same
        corridor. They are caps: a marker seen at 8.2 m stops the creep
        there. The two 2.20 m numbers are measured from the CENTRE of the
        boxes, which themselves sit within about +/-30 cm of it, so the
        sidestep lands the aircraft in the marker's camera footprint and
        LAND_ALIGN takes out the rest.

        thermal_drop_sitl.launch.py passes DIFFERENT values for all four:
        the scaled arena is not the real arena's geometry multiplied, it is
        its own layout. Do not reconcile them.

    It needs the downward camera, so aruco defaults to true as well and
    aruco_min_marker_distance_rate to 0.02 (see that argument for why the
    OpenCV default detects nothing over a marker on a pad). Nothing has to
    be passed:

        ros2 launch drone_testing thermal_drop.launch.py agent_only:=false

    aruco_image_topic stays EMPTY on the aircraft, so aruco_pose opens
    /dev/video<aruco_camera_index> itself. /camera/down/image_raw is a
    Gazebo topic and exists only in the simulator.

    Everything in this file still applies: thermal_fsm IS a thermal_drop
    with two ends bolted on, and reads every parameter here.

    The drop ALONE -- survey from wherever you took off, no markers:

        ros2 launch drone_testing thermal_drop.launch.py fsm:=false \\
            aruco:=false agent_only:=false

    In the simulator all of that is set for you:

        ros2 launch drone_testing thermal_drop_sitl.launch.py

    which also swaps thermal_sensor (MLX90640 over I2C) for thermal_sim
    (the Gazebo thermal camera) and scales every length by 2.2. Read that
    file's header for the arena, the scaling rule and the flight plan.

THE SERVO

    Test it on its own first -- the mission is a slow way to debug a servo:

        ros2 run drone_testing servo_test --ros-args -p command:=sweep

    That sends MAV_CMD_ACTUATOR_TEST across Servo 1-8, one a second, and
    prints PX4's ack for each. Watch the servo, note the number printed for
    the step that moved it -- pass that number as-is, it is already in the
    encoding PX4 wants, and it is NOT the "Servo 4" from QGC's Actuators tab
    (param5 is a MAVLink ACTUATOR_OUTPUT_FUNCTION: Servo 1-8 is 33-40, or
    1201-1208 in PX4's own numbering). Then:

        mode:=dryrun servo_command:=actuator_test servo_function:=<n>

    Two commands, and they are not interchangeable:

      set_actuator (default)  MAV_CMD_DO_SET_ACTUATOR, for FLIGHT. Needs the
                              output on "Offboard Actuator Set <servo_index>"
                              in QGC -> Actuators -- "RC AUX 1" is an RC
                              passthrough and ignores it. PX4 applies these
                              values only while ARMED, so on a disarmed bench
                              it can be accepted and still move nothing.
      actuator_test           What the QGC Actuator sliders send, and the one
                              that works DISARMED. Addressed by FUNCTION
                              number, so set servo_function too -- and PX4
                              DENIES it outright if the vehicle is armed, if a
                              safety button is fitted and unpressed, or if
                              COM_MOT_TEST_EN is not 1.

    First flight: release_enabled:=false. The whole mission runs and the drop
    is logged but the servo never moves.

WATCHING THE THERMAL CAMERA

    http://<jetson>:8082/  -- the live 32x24 frame, upscaled and false-
    coloured, with a thick green box on the HOTTEST blob (the one the mission
    would fly to), a yellow cross on the exact point it aims at, thin grey
    boxes on every other warm blob it is rejecting, and a white crosshair at
    frame centre (straight down). Bring the green box onto the crosshair and
    that is the drop lined up. http://<jetson>:8082/snapshot for one JPEG.

Watch:  ros2 topic echo /thermal/hotspot      (sensor alive?)
        ros2 topic echo /thermal_drop/target  (box_n|box_e|err|stage)
        ros2 topic echo /thermal_drop/ready   (true once released)
        ros2 topic echo /led/command          (blink_red during the descent)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    L = LaunchConfiguration
    bench = PythonExpression(["'", L('mode'), "' == 'bench'"])
    # The SERIAL uXRCE-DDS agent. It runs when it is asked for AND the mode is
    # not bench. Both halves matter -- see the 'agent' argument below.
    serial_agent = PythonExpression(
        ["'", L('agent'), "'.lower() not in ('false', '0') and '",
         L('mode'), "' != 'bench'"])
    # The release node runs when it is asked for AND the release is armed.
    # release_enabled:=false means "fly it all but never move the servo", and
    # a release node sitting there ready to move it would be exactly that
    # promise broken.
    servo_owns_output = PythonExpression(
        ["'", L('servo_node'), "'.lower() not in ('false', '0') and '",
         L('release_enabled'), "'.lower() not in ('false', '0')"])

    args = [
        ('agent_only', 'true', 'Start agent + sensor but not the flight node.'),
        # THE SERIAL AGENT, AND WHY IT HAS AN OFF SWITCH.
        #
        #   On the aircraft the Jetson talks to the Pixhawk over
        #   /dev/ttyTHS1, and micro_ros_agent is what carries the uORB
        #   topics across it. In a SIMULATION there is no serial link and no
        #   Pixhawk: thermal_drop_sitl.launch.py runs its own
        #   `MicroXRCEAgent udp4 -p 8888` instead, and micro_ros_agent is
        #   not even installed on a desktop.
        #
        #   Without this switch, including this file from the SITL launch
        #   threw
        #       [ERROR] [launch]: Caught exception in launch:
        #       "package 'micro_ros_agent' not found"
        #   40 s into the run -- and because that exception happens INSIDE
        #   an IncludeLaunchDescription, launch tears the whole session down
        #   with it: Gazebo, PX4, the bridge and the UDP agent all took a
        #   SIGINT and the simulation "randomly exited" with the arena
        #   loaded and the aircraft still on the pad.
        #
        #   course_mission.launch.py has exactly this argument, for exactly
        #   this reason, and course_mission_sitl passes agent:=false.
        ('agent', 'true',
         'Start micro_ros_agent on the serial link to the Pixhawk. false in '
         'any simulation -- there is no serial link there, the SITL launch '
         'runs its own UDP agent, and the package is not installed on a '
         'desktop, which takes the whole launch down with it.'),
        ('mode', 'fly',
         'bench = log only. dryrun = the whole mission on a simulated vehicle '
         '(no arming, no setpoints, motors cannot spin) with a REAL thermal '
         'feed, LED and servo. fly = the real thing.'),

        # ---- sensor ----
        ('refresh_hz', '8', 'MLX90640 refresh: 1/2/4/8/16. 8 is the useful max on I2C.'),
        ('publish_preview', 'false', 'Also publish the annotated view as a ROS topic.'),
        ('stream_port', '8082',
         'Browser view of the thermal camera: http://<jetson>:8082/. 0 disables. '
         'NOT 8080/8081 -- window_detect and bar_detect own those.'),
        ('jpeg_quality', '70', ''),
        ('hfov_deg', '110.0', ''),
        ('vfov_deg', '75.0', ''),
        ('cam_yaw_deg', '0.0', 'Image-top rotation from the nose, about the down axis.'),
        ('flip_lr', 'false', ''),
        ('flip_ud', 'false', ''),
        ('min_contrast', '3.0', 'deg C above the frame median to count as a blob.'),
        ('min_blob_pixels', '2', 'A one-pixel hot spot is noise, not a box.'),

        # ---- offsets ----
        ('cam_from_drop_forward', '0.0',
         'm the CAMERA sits FORWARD of the drop point (release centre).'),
        ('cam_from_drop_right', '0.0',
         'm the CAMERA sits RIGHT of the drop point.'),
        ('drop_from_cog_forward', '0.0', 'm the drop point sits forward of the CoG.'),
        ('drop_from_cog_right', '0.0', 'm the drop point sits right of the CoG.'),

        # ---- arena ----
        ('box_height', '0.0', 'm, height of the box tops above the floor.'),
        ('expected_boxes', '3', ''),
        ('search_radius', '3.0',
         'm from the survey point; blobs further out are ignored outright. '
         'It was 1.5 m when the survey was a RING, where the moving centre '
         'kept coming back within 1.5 m of each box in turn. The survey is '
         'now ONE point that has to see all three at once, and from 2.20 m '
         'right of the marker the far box is close to 3 m away -- at 1.5 m '
         'the mission would quietly rank only the nearest box and drop on '
         'it, with nothing in the log to say the others were thrown away. '
         'Keep it just big enough to cover the box cluster: it is also what '
         'rejects a radiator, a lamp or a person at the edge of frame.'),
        ('cluster_radius', '0.30', ''),
        ('min_hot_margin', '2.0', 'deg C the hottest box should lead by (warning only).'),
        ('verify_frames', '8', 'Frames of evidence before the descent may start.'),
        ('verify_window', '3.0', 's the hottest-in-frame evidence is counted over.'),
        ('verify_ratio', '0.7', 'Fraction of those frames the box must be hottest in.'),
        ('retarget_margin', '1.5',
         'deg C another blob must beat the target by, consistently, to steal it.'),

        # ---- flight ----
        ('cruise_altitude', '1.8',
         'm the aircraft takes off to and flies the WHOLE outbound mission '
         'at: the marker creep, the hover on the marker and the sidestep '
         'onto the boxes. Low on purpose -- all three are looking for a '
         'marker on the FLOOR, and low means more pixels on it and a '
         'smaller footprint, so a marker in frame is a marker nearly '
         'underneath. The climb to survey_altitude is paid for once, over '
         'the boxes, where the aircraft is stationary anyway.'),
        ('survey_altitude', '2.5',
         'm the boxes are watched from, clamped to max_survey_altitude in the '
         'node. High enough that all three boxes are in ONE thermal frame, '
         'which is what lets the survey be a single observation instead of a '
         'flown pattern. It is NOT the height the rest of the mission flies '
         'at -- see cruise_altitude. The aircraft climbs to this once it is '
         'over the boxes and descends from it to drop_altitude.'),
        ('drop_altitude', '0.5', 'm above the floor, floored at 0.5 in the node.'),
        ('survey_dwell_seconds', '4.0',
         's held still at the survey point, watching. A TIME, not a length.'),
        ('survey_step', '0.0',
         'm. 0 = NO SEARCH PATTERN: stop over the boxes, watch, take the '
         'hottest. A positive value puts the old four-point ring back, for an '
         'arena where the boxes do not fit in one frame.'),
        ('survey_all_points', 'false',
         'true = always fly the whole ring. Only means anything with '
         'survey_step > 0.'),
        ('approach_tolerance', '0.15', ''),
        ('descend_tolerance', '0.12', 'm; descent pauses while further off than this.'),
        ('descend_speed', '0.82', ''),
        ('align_tolerance', '0.08', 'm at drop height to confirm the drop.'),
        ('align_settle_seconds', '2.0', ''),
        ('hover_seconds', '2.0', 's of settling after the release, before climbing.'),

        # ---- the drop and the exit ----
        ('release_enabled', 'true', 'false = fly it all but never move the servo.'),
        ('servo_index', '1', 'PX4 "Offboard Actuator Set N" the servo is on.'),
        ('servo_drop_value', '1.0', 'Actuator value that opens it, -1..1.'),
        ('servo_neutral_value', '-1.0', 'Actuator value it returns to.'),
        ('servo_hold_seconds', '2.0', 's the servo is held open.'),
        ('servo_command', 'set_actuator',
         'set_actuator = MAV_CMD_DO_SET_ACTUATOR (needs the output on '
         '"Offboard Actuator Set N"; PX4 may ignore it while disarmed). '
         'actuator_test = what QGC\'s Actuators tab uses, which DOES work '
         'disarmed -- then set servo_function too.'),
        ('servo_function', '0',
         'PX4 output FUNCTION number of the servo, for servo_command:='
         'actuator_test. Read it off the Actuators tab; 0 means unset.'),
        ('servo_test_on_start', 'false',
         'Dry run only: open and close the servo once at startup.'),
        # WHO MOVES THE SERVO. Exactly one of the two, always.
        ('servo_node', 'true',
         'Run servo_controller.py, the release node, and let IT drive the '
         'output. The flight node then only says WHEN, by publishing on '
         'drop_trigger_topic. false = the flight node commands the actuator '
         'itself, which is what the SITL does (there is no servo in Gazebo).'),
        ('drop_trigger_topic', '/servo/drop',
         'std_msgs/Bool. True the instant the drop commits, False once the '
         'payload has had servo_hold_seconds to clear. Bench-testable on its '
         'own:  ros2 topic pub --once /servo/drop std_msgs/Bool "data: true"'),
        ('servo_close_on_arm', 'true',
         'servo_controller only: assert the bay CLOSED for a couple of '
         'seconds the moment the vehicle ARMS. THIS IS NOT THE SAME AS '
         'servo_close_on_start, and covers what that one cannot. While '
         'disarmed PX4 drives the output to the Actuators tab\'s Disarmed '
         'value (2000 us here = the -1.0 end = closed) with nothing being '
         'sent at all. At the instant of arming PX4 stops using that value '
         'and uses the actuator set\'s control value -- which, until the '
         'first drop, NOTHING HAS EVER SENT. If PX4 defaults it to 0.0 then '
         'with Min 1000 / Max 2000 that is 1500 us, i.e. HALF OPEN, on the '
         'pad, with the cone loaded. servo_close_on_start cannot help: it '
         'fires while the vehicle is still disarmed, and PX4 IGNORES '
         'DO_SET_ACTUATOR while disarmed, so the command is discarded. A '
         'BURST and not a stream -- see close_on_arm_seconds.'),
        ('servo_close_on_arm_seconds', '2.0',
         's the neutral value is re-sent after arming, then it stops. PX4 '
         'latches the last DO_SET_ACTUATOR, so this only has to land once; a '
         'permanent stream would put a vehicle_command into PX4\'s queue '
         'several times a second for the whole flight, and that queue '
         'overruns -- which is why OffboardSequence throttles its own '
         'commands. A dropped arm or offboard-mode command is a worse '
         'failure than the one this prevents.'),
        ('servo_close_on_start', 'false',
         'servo_controller only: send neutral once at startup so the bay is '
         'known-closed. Off by default -- on a loaded, armed vehicle an '
         'unasked-for servo command is not a courtesy.'),
        ('sim_descend_speed', '0.95', 'm/s the SIMULATED vehicle descends at.'),
        ('retreat_altitude', '1.2', 'm climbed back to after the drop.'),
        ('retreat_right', '2.2',
         'm stepped to the RIGHT after the drop, off the centre of the boxes '
         'and onto the corridor the datum marker is on. MEASURED in the real '
         'arena, where the boxes themselves sit within about +/-30 cm of that '
         'centre, so the step lands the aircraft within the datum marker\'s '
         'camera footprint rather than exactly on it -- which is what '
         'land_search_distance and then LAND_ALIGN are for.'),
        ('land_after_drop', 'true', 'false = hold clear of the box instead.'),
        ('track_gate', '0.40', ''),
        ('flight_seconds', '150.0', ''),
        ('hold_seconds', '4.0', ''),
        ('ground_wait_seconds', '5.0', ''),
        ('climb_speed', '0.35', ''),
        ('land_speed', '0.15', ''),
        ('move_speed', '0.80',
         'm/s the carrot is walked at. Binds only while it is below '
         'MPC_XY_P * move_leash -- see move_leash.'),
        ('request_offboard_from_ros', 'true',
         'false = you flip the Offboard switch on the TX.'),

        ('led', 'true', 'Run the WS2812B status light node.'),
        ('num_pixels', '5', ''),
        ('blink_hz', '2.0', 'Red blink rate during the descent.'),

        ('flight_node_delay', '8.0', ''),

        # ---- how the carrot is walked (OffboardSequence) ----
        # move_leash is the one that actually governs how fast the aircraft
        # moves: the commanded x/y is capped this far ahead of the MEASURED
        # x/y and PX4 flies that capped error, so the ground speed is roughly
        # MPC_XY_P * move_leash whatever the speed arguments say. It is a
        # world distance, so a scaled arena scales it.
        ('move_leash', '2.00',
         'm the commanded x/y may lead the measured x/y. THE HORIZONTAL '
         'SPEED CEILING, and it is set by the flight controller: PX4 flies '
         'the capped position error at roughly MPC_XY_P * move_leash, so no '
         'speed argument in this file can beat that product. THIS AIRCRAFT '
         'RUNS MPC_XY_P = 0.5, which the ARK Flow documentation asks for -- '
         'so the original 0.40 m leash capped every leg at 0.5 * 0.40 = '
         '0.20 m/s whatever move_speed said, and a 9 m creep took 45 s of a '
         '60 s stage timeout. 2.00 m lifts the ceiling to 1.00 m/s, which '
         'leaves move_speed (0.80) as the binding limit instead -- and that '
         'is the RIGHT way round: while the ramp is the slower of the two the '
         'position error stays well inside the leash, so there is no standing '
         'error for PX4 to wind up on and pay back as overshoot. Re-derive it '
         'if MPC_XY_P changes: leash > wanted_speed / MPC_XY_P. NOTE it is '
         'also how far a vehicle whose ESTIMATE has frozen may be dragged '
         'before anything notices, and at 2.00 m that is no longer a small '
         'number -- leg_stall_seconds is what now catches that case, and it '
         'is the reason this leash may safely be this long.'),
        ('min_altitude', '0.4', ''),
        ('max_altitude', '3.0', ''),
        ('takeoff_accept_tolerance', '0.30', ''),
        ('stage_timeout', '45.0', ''),
        ('retreat_timeout', '30.0', ''),

        # ---- the blob -> NED projection ----
        ('frame_latency', '0.06',
         's between the attitude a frame was taken at and its timestamp.'),
        ('max_ray_angle_deg', '60.0',
         'Beyond this a pixel ray is too grazing to intersect the floor plane '
         'reliably and the blob is dropped.'),
        ('min_cluster_frames', '3',
         'Frames a survey cluster needs before it counts as a box.'),

        # ================= THE FULL MISSION (thermal_fsm) =================
        #
        # fsm:=true swaps the flight node from thermal_drop -- which climbs,
        # surveys whatever is under it, drops and lands -- for thermal_fsm,
        # which flies the WHOLE mission: forward to an ArUco marker, one step
        # right onto the boxes, then the same survey/drop/retreat, then a
        # precision landing on the next marker. Everything above still
        # applies; thermal_fsm IS a thermal_drop with two ends bolted on.
        ('fsm', 'true',
         'true (THE DEFAULT) runs thermal_fsm: marker -> boxes -> drop -> '
         'marker -> land, the whole competition run. false drops back to '
         'thermal_drop, which surveys from wherever it took off.'),

        # ---- the legs thermal_fsm adds. REAL course metres. ----
        ('mark_search', 'true',
         'false skips the marker hunt and surveys from the takeoff point, '
         'which is thermal_drop\'s own mission.'),
        ('mark_required', 'true',
         'true = a marker that is never found ENDS the mission. false = '
         'survey from wherever the creep gave up (normally bare floor).'),
        ('mark_search_distance', '9.0',
         'm of forward creep before the marker hunt gives up. MEASURED: the '
         'takeoff marker and the one in front of it are 8.7-8.8 m apart in '
         'the real arena, so this is that plus slack for where the aircraft '
         'actually left the pad. It is a CAP, not a leg -- a marker seen at '
         '8.2 m stops the creep there. Reaching it means the marker is not '
         'there, and mark_required says what to do about that.'),
        ('mark_search_speed', '0.70',
         'm/s of forward creep. Held BELOW move_speed on purpose: this leg is '
         'the one looking for a marker, and the limits on it are the camera '
         'and the flow, not the position controller. At cruise_altitude '
         '1.80 m the down camera footprint is about 2 m, so 0.70 m/s crosses '
         'it in under 3 s -- CHECK THE DETECTOR RATE against that. aruco_pose '
         'must publish at 10 Hz or better for a marker to be seen in enough '
         'frames to be believed; if it runs at 5 Hz this leg gets about 14 '
         'looks and at 2 Hz it gets 6, and a marker flown over between two '
         'ticks was never there as far as the mission is concerned.'),
        ('mark_min_travel', '2.00',
         'm that must be flown before a marker counts -- the aircraft arms ON '
         'a marked pad and must not "find" the one it is standing on. Raised '
         'from 1.00 m with cruise_altitude: the down camera footprint is '
         'about 2 m across at 1.80 m, so the pad marker stays in frame for '
         'roughly 1 m past it and 1.00 m of travel left no margin at all -- '
         'a little drift, or an `along` that under-read, and the aircraft '
         '"found" the pad it had just left and stopped 1 m into a 9 m leg.'),
        ('mark_hover_seconds', '1.0',
         's stationary over the marker. A TIME, not a length: never scaled.'),
        ('box_offset_right', '2.20',
         'm RIGHT of the marker, which is where the boxes are.'),
        ('mark_stage_timeout', '90.0',
         's. Generous on purpose. It is meant to catch a leg that ran out of '
         'arena, and a leg that merely ran SLOW must not trip it: at the old '
         '0.20 m/s effective speed a 9 m creep needed 45 s of the old 60 s, '
         'and any headwind or reacquisition spent the rest. Flow stalls no '
         'longer spend it at all -- see leg_stall_seconds.'),
        # NOTE leg_lookahead (3.20 m) is now LONGER than the box_offset and
        # retreat_right legs (2.20 m). On those the carrot clamps to the far
        # end immediately and the leg degenerates to point-to-point, which is
        # what it always was before the line-following rewrite and is fine
        # over 2 m -- the cross-track argument in LEG_LOOKAHEAD is about the
        # 9 m corridor legs, where the carrot still sits on the line.
        ('leg_lookahead', '3.20',
         'm ahead ALONG the line the carrot is placed. Every straight leg is '
         'flown by holding a LINE, not by aiming at a point at the far end: '
         'a far-end target corrects a cross-track error e with d to run by '
         'only atan(e/d), which over a 21 m leg is nothing, and the aircraft '
         'drifts. Must stay LARGER than move_leash or the leash clamps the '
         'carrot and the direction stops meaning anything.'),
        ('leg_bearing_deg', 'nan',
         'The corridor\'s ABSOLUTE bearing in the estimator\'s NED frame, '
         'degrees. nan = measure it from the settled heading instead, which '
         'is right on the aircraft (the operator aims the nose and nothing '
         'surveys the hall). Give it a number where the layout IS known: '
         'EKF2 datums yaw off the magnetometer and x/y off flow, so the two '
         'do not share a north, and in the scaled arena that offset was a '
         'steady 5.95 deg -- 2.40 m of sideways error over a 23 m leg flown '
         'perfectly straight.'),
        # ---- following the painted floor line, and the end of the corridor ----
        ('line_follow', 'false',
         'true = correct the corridor legs off the high-contrast line painted '
         'on the arena floor, seen by the DOWNWARD camera. OFF BY DEFAULT: '
         'the fixed-bearing leg is what has been flown, and this is the thing '
         'being added. When the line is in view the carrot is put on it '
         'instead of on the bearing latched at the start of the leg; when it '
         'is lost the leg carries on exactly as it does today, so the failure '
         'mode of the new behaviour is the old behaviour. Needs the line node '
         '(line:=true), which needs aruco_publish_frames because one v4l2 '
         'device cannot be opened twice.'),
        ('line', 'false', 'Run line_detect. Implied by nothing -- set it.'),
        ('line_gain', '1.0',
         'How much of the measured cross-track is taken out per carrot.'),
        ('line_max_nudge', '0.60',
         'm. The largest sideways correction ONE frame may ask for. This is '
         'the safety argument for line following: a floor seam, a cable or '
         'the edge of a mat mistaken for the corridor cannot bend the course '
         'further than this, and the next good frame undoes it.'),
        ('line_start_after', '2.0',
         'm of a leg that must be flown before the strip detector is believed '
         'at all. THE PAD IS THE PROBLEM: straight after takeoff the down '
         'camera sees mostly ArUco MARKER, and a marker is a bright thing on '
         'a dark floor, which is exactly what the strip detector segments. It '
         'would report the marker\'s own axis as the corridor. The same '
         'applies leaving the datum marker on the way home, so the same gate '
         'covers both. Line following is ALSO suppressed outright whenever '
         '/aruco/detected is true, which is the belt to this braces.'),
        ('line_lost_seconds', '1.0', 's a line fix stays usable.'),
        ('line_min_quality', '0.35',
         '0..1. The fraction of detected line-segment length that had to vote '
         'for the winning orientation. Below this the detector is guessing.'),
        ('line_max_heading_deg', '35.0',
         'deg. A line further off the nose than this is not the corridor -- '
         'it is a wall join or a shadow -- and is ignored rather than chased.'),
        ('line_stream_port', '8084',
         'Browser view of the line detection. 0 = off. USE IT before flying: '
         'walking the aircraft left must make the cross-track go POSITIVE.'),
        ('line_cam_yaw_deg', '0.0',
         'Image-top rotation of the DOWN camera from the nose, about the down '
         'axis. NOT cam_yaw_deg, which is the THERMAL camera on its own mount.'),
        ('line_flip_lr', 'false',
         'Set if the line detector reports LEFT and RIGHT swapped.'),
        ('line_flip_ud', 'false', 'Set if FORWARD and BACK come out swapped.'),
        ('line_blur', '7', 'Gaussian kernel, odd.'),
        ('line_close_ksize', '15',
         'Morphological CLOSE, pixels. Must be big enough to swallow the '
         'largest dark leaf in the print, so the strip becomes ONE blob '
         'instead of a constellation of white gaps between leaves.'),
        ('line_open_ksize', '9',
         'Morphological OPEN, pixels. Removes the speckle the granite throws.'),
        ('line_strip_is_bright', 'true',
         'The strip is BRIGHTER than the floor. True for printed material on '
         'dark granite; set false if you ever lay a dark strip on a pale '
         'floor, or the detector will lock onto the floor instead.'),
        ('line_min_area_frac', '0.05',
         'Smallest fraction of the frame the strip may occupy and still count.'),
        ('line_max_area_frac', '0.85',
         'Largest. Above this the "strip" is the whole frame, which means the '
         'threshold split something other than strip-from-floor.'),
        ('line_min_elongation', '1.6',
         'How long-and-thin the blob must be. A corridor strip crossing the '
         'frame is elongated; a sunlit patch, a mat or the takeoff pad is '
         'round-ish, and this is what rejects them.'),
        ('line_downscale', '0.5',
         'Shrink each frame before detecting. Halving the side quarters the '
         'work, and a painted corridor line does not need full resolution.'),

        ('wall_stop', 'false',
         'true = MARK_SEARCH also ends when the ZED sees the end of the '
         'corridor within wall_stop_distance. Treated as ARRIVAL: the '
         'sidestep to the boxes is then measured from where the aircraft '
         'stopped. NOTE that box_offset_right was measured from the MARKER, '
         'so this substitutes one datum for another and inherits the '
         'difference. It is the right ending when the marker was missed; it '
         'is not as good as the marker, and the log says which one was used.'),
        ('wall', 'false', 'Run wall_watch (and so require the ZED).'),
        ('wall_stop_distance', '1.50',
         'm. The closest the aircraft may come to the thing in front.'),
        ('wall_lost_seconds', '1.0',
         's a clearance reading stays usable. A dead depth topic must not '
         'read as a corridor with no end -- but it must not stop the mission '
         'either, so a stale reading simply does not fire the stop.'),
        ('wall_depth_topic', '/zed/zed_node/depth/depth_registered', ''),
        ('wall_camera_info_topic', '/zed/zed_node/depth/camera_info', ''),
        ('wall_min_pixels', '300',
         'How many non-floor pixels must agree before anything is called an '
         'obstacle. The thing at the end of the run is a BOX and does not '
         'fill the frame, so this counts rather than taking a minimum.'),
        ('wall_floor_margin', '0.35',
         'm above the floor below which a depth point IS the floor and is '
         'thrown away. THE IMPORTANT ONE: the aircraft pitches ~10 deg to '
         'hold 0.8 m/s, which swings the floor into the bottom of the ZED\'s '
         'frame a couple of metres ahead, and without this rejection the '
         'mission stops in mid-arena -- and does it MORE the faster it flies.'),
        ('wall_camera_pitch_deg', '0.0',
         'deg the ZED is tilted DOWN on the airframe, if it is.'),

        ('aruco_publish_frames', '',
         'Topic for aruco_pose to re-publish its frames on, so line_detect '
         'can see the same pixels. Empty = publish nothing. Set '
         'automatically when line:=true.'),

        ('leg_stall_seconds', '4.0',
         's of unhealthy optical flow, on a straight leg, before the leg is '
         'abandoned and the aircraft lands. Every leg ends on the ESTIMATED '
         'distance along the line, and the carrot is leashed to the estimate '
         'too, so when flow stops correcting the aircraft stops dead and the '
         'leg stops counting -- while the stage clock keeps running and '
         'eventually lands it, blaming a marker it never reached. The stage '
         'clock is now HELD for the whole stall, so a stall that clears costs '
         'nothing; this is how long one may last before it is called.'),
        ('leg_trim_deg', '0.0',
         'deg added to every leg heading, +ve to the RIGHT. For a KNOWN, '
         'repeatable aim bias only. Leave at 0 until a leg has been flown '
         'and measured; the log prints the cross-track error of each one.'),

        # ---- the precision landing after the drop ----
        ('precision_land', 'true',
         'false = plain PX4 land after the retreat, as thermal_drop does.'),
        ('land_search_distance', '2.50',
         'm of forward creep after the retreat, looking for the DATUM marker. '
         'Short on purpose: retreat_right is supposed to have landed on it, '
         'so this is an acquisition allowance, not a search.'),
        ('land_search_speed', '0.70', ''),
        ('land_return', 'true',
         'true = the marker found after the sidestep is a DATUM, not the pad: '
         'align on it, then fly the corridor BACKWARDS to the landing marker '
         '8.7-8.8 m away. That is the arena as laid out. false = land on the '
         'first marker found, for a bench or a one-marker test.'),
        ('land_return_distance', '9.0',
         'm of backward creep before the run home gives up. The same '
         '8.7-8.8 m plus slack as mark_search_distance, because it is the '
         'same pair of markers. Reaching it is NOT a failure -- the cone is '
         'already in the box -- so the aircraft simply lands where it is.'),
        ('land_return_speed', '0.70',
         'm/s backwards. As slow as the outbound creep: a marker that crosses '
         'the frame between two detector ticks was never seen.'),
        ('land_return_min_travel', '1.00',
         'm that must be flown before a marker counts on the way home. '
         'Without it the datum the aircraft is sitting over is instantly '
         '"found" again and it lands at the wrong end of the arena.'),
        ('pad_centre_tolerance', '0.10', 'm off the marker that counts as centred.'),
        ('pad_centre_seconds', '1.0', 's it must stay there before descending.'),
        ('pad_descent_rate', '0.20', 'm/s the setpoint walks down.'),
        ('pad_handoff_height', '0.45',
         'm at which PX4\'s land takes over. Below this the marker no longer '
         'fits in the frame.'),
        ('pad_gain', '0.8', 'Fraction of the measured error moved per tick.'),
        ('pad_max_nudge', '0.30', 'm, the largest single step onto the marker.'),
        ('pad_lost_seconds', '2.0', 's a marker pose stays usable.'),
        ('pad_stage_timeout', '45.0', ''),
        ('marker_anchor_gain', '0.35',
         'How much of each new marker fix goes into the ANCHOR -- the pad\'s '
         'position in NED, which the landing flies to instead of chasing the '
         'newest frame. Lower = steadier and laggier.'),
        ('marker_anchor_max_age', '3.0',
         's the anchor stays usable with no new fix. Longer than '
         'pad_lost_seconds on purpose: losing SIGHT of the pad must not lose '
         'the PLACE, which is what lets the descent carry on through the '
         'blinks that ground effect and a pad overflowing the frame cause.'),
        ('ground_effect_height', '1.20',
         'm AGL below which the airframe is in its own downwash. Under it '
         'the descent halves its rate and the centring gate is RELAXED: '
         'chasing a wobble that is not a real position error is what makes '
         'an aircraft hunt in the last metre.'),
        ('max_survey_altitude', '2.6',
         'm, the clamp on survey_altitude. It was 1.6 when the survey was a '
         'ring flown 1.5 m up; the survey is now one observation from 2.5 m, '
         'so the clamp has to clear that. 2.5 m IS near the MLX90640\'s '
         'limit -- a 30 cm box is about 4 px across there -- so if the boxes '
         'come back as one blob or as none, lower survey_altitude rather than '
         'loosening min_contrast or min_blob_pixels.'),

        # ---- the downward ArUco detector the marker stages fly on ----
        ('aruco', 'true',
         'Start aruco_pose on the downward camera. Required by fsm:=true -- '
         'which is now the default -- unless something else is already '
         'publishing /aruco/point. Its view is http://<jetson>:8083/.'),
        ('aruco_camera_index', '0', 'v4l2 device. Ignored if aruco_image_topic is set.'),
        ('aruco_image_topic', '',
         'Take frames from a ROS topic instead of a camera device. This is '
         'how it runs in the simulator: /camera/down/image_raw.'),
        ('aruco_marker_ids', '2,3',
         'The markers that COUNT, comma separated. The takeoff pad\'s id is '
         'deliberately not among them -- see mark_min_travel.'),
        ('aruco_marker_size', '0.40', 'm, edge length including the black border.'),
        ('aruco_hfov_deg', '90.0',
         'Must match the downward camera. In the sim that is down_cam.xacro\'s '
         '1.5708 rad.'),
        ('aruco_dict', 'DICT_5X5_50', ''),
        ('aruco_stream_port', '8083',
         'Browser view of the downward camera. 8082 is the thermal one.'),
        ('aruco_min_marker_distance_rate', '0.02',
         'cv2.aruco merges two candidate quads whose corners are within this '
         'fraction of the image and keeps the LARGER. A marker lying on a '
         'square PAD has the pad\'s outline as a second, concentric '
         'candidate, and at cv2.aruco\'s own default of 0.05 it swallows the '
         'marker and NOTHING is ever detected. 0.02 is what the simulator '
         'flies and what is defaulted here. 0 is not "off" -- it detects '
         'nothing either.'),

        # ---- the simulator's thermal source ----
        ('thermal_sim', 'false',
         'true runs thermal_sim (Gazebo thermal camera -> /thermal/image) '
         'instead of thermal_sensor (MLX90640 over I2C). There is no I2C in '
         'a simulation and no Gazebo on the aircraft; nothing downstream can '
         'tell the two apart.'),
        ('thermal_raw_topic', 'thermal/raw',
         'The bridged Gazebo thermal camera, for thermal_sim.'),
        ('thermal_resolution', '0.01',
         'KELVIN PER COUNT. Must equal <resolution> in thermal_cam.xacro.'),
        ('thermal_noise_c', '0.1',
         'deg C of simulated sensor noise, 1 sigma. The MLX90640\'s NETD.'),
    ]
    declared = [DeclareLaunchArgument(n, default_value=d, description=h)
                for n, d, h in args]

    microxrce = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_xrce_dds_agent', output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
        condition=IfCondition(serial_agent),
    )

    sensor = Node(
        package='drone_testing', executable='thermal_sensor', name='thermal_sensor',
        output='screen', emulate_tty=True,
        parameters=[{'refresh_hz': L('refresh_hz'),
                     'publish_preview': L('publish_preview'),
                     'stream_port': L('stream_port'),
                     'jpeg_quality': L('jpeg_quality'),
                     # The overlay must use the flight node's thresholds, or
                     # the boxes on the stream are not the boxes it flies to.
                     'min_contrast': L('min_contrast'),
                     'min_blob_pixels': L('min_blob_pixels')}],
        condition=UnlessCondition(L('thermal_sim')),
    )

    # THE RELEASE. One node, one output: it waits for True on
    # drop_trigger_topic and opens the servo. It is a separate process from
    # the flight node on purpose -- the release can be run, watched and
    # bench-tested by itself, with the aircraft on the table and no mission
    # in the air. release_via_servo_node below is the other half: it stops
    # the flight node commanding the same output, because two publishers
    # sending different values to one actuator at 20 Hz is a servo that
    # buzzes rather than one that opens.
    servo_node = Node(
        package='drone_testing', executable='servo_controller',
        name='servo_controller', output='screen', emulate_tty=True,
        parameters=[{'drop_trigger_topic': L('drop_trigger_topic'),
                     'servo_index': L('servo_index'),
                     'servo_drop_value': L('servo_drop_value'),
                     'servo_neutral_value': L('servo_neutral_value'),
                     'servo_hold_seconds': L('servo_hold_seconds'),
                     'servo_command': L('servo_command'),
                     'servo_function': L('servo_function'),
                     'close_on_start': L('servo_close_on_start'),
                     'close_on_arm': L('servo_close_on_arm'),
                     'close_on_arm_seconds': L('servo_close_on_arm_seconds')}],
        condition=IfCondition(servo_owns_output),
    )

    # The simulator's stand-in for it: same topic, same encoding, same
    # annotated stream on the same port, same find_blobs. See thermal_sim.py.
    sensor_sim = Node(
        package='drone_testing', executable='thermal_sim', name='thermal_sim',
        output='screen', emulate_tty=True,
        parameters=[{'raw_topic': L('thermal_raw_topic'),
                     'resolution': L('thermal_resolution'),
                     'noise_c': L('thermal_noise_c'),
                     'publish_preview': L('publish_preview'),
                     'stream_port': L('stream_port'),
                     'jpeg_quality': L('jpeg_quality'),
                     'min_contrast': L('min_contrast'),
                     'min_blob_pixels': L('min_blob_pixels')}],
        condition=IfCondition(L('thermal_sim')),
    )

    # line_detect needs the frames aruco_pose captures. Rather than make the
    # operator remember to set two arguments that must agree, line:=true
    # implies the topic -- and an explicit aruco_publish_frames still wins, for
    # the case where the frames are wanted without the line node.
    frames_topic = PythonExpression(
        ["'", L('aruco_publish_frames'), "' or ('/aruco/image' if '",
         L('line'), "'.lower() not in ('false', '0') else '')"])

    # The downward ArUco detector. thermal_fsm's MARK_* and LAND_* stages fly
    # on /aruco/detected and /aruco/point and on nothing else, so with
    # aruco:=false those stages find nothing, creep their full distance and
    # give up -- which is a legible outcome, not a crash.
    aruco_node = Node(
        package='drone_testing', executable='aruco_pose', name='aruco_pose',
        output='screen', emulate_tty=True,
        parameters=[{'camera_index': L('aruco_camera_index'),
                     'image_topic': L('aruco_image_topic'),
                     'marker_ids': L('aruco_marker_ids'),
                     'marker_size': L('aruco_marker_size'),
                     'hfov_deg': L('aruco_hfov_deg'),
                     'aruco_dict': L('aruco_dict'),
                     'min_marker_distance_rate':
                         L('aruco_min_marker_distance_rate'),
                     'stream_port': L('aruco_stream_port'),
                     # One v4l2 device, two detectors: the owner shares the
                     # frames rather than the other one prising the device
                     # open. Empty unless line:=true.
                     'publish_frames': frames_topic}],
        condition=IfCondition(L('aruco')),
    )

    # The floor-line detector. It does NOT open the camera -- it takes the
    # frames aruco_pose re-publishes, so both detectors see the same pixels,
    # which is also what you want when deciding whether the marker or the line
    # was the thing that lied.
    line_node = Node(
        package='drone_testing', executable='line_detect', name='line_detect',
        output='screen', emulate_tty=True,
        parameters=[{'image_topic': frames_topic,
                     'hfov_deg': L('aruco_hfov_deg'),
                     # ITS OWN dials, NOT cam_yaw_deg/flip_lr/flip_ud --
                     # those describe how the THERMAL camera is mounted, and
                     # this node is reading the DOWN camera. The two are
                     # different cameras on different mounts, and sharing the
                     # numbers would silently rotate the line geometry by
                     # whatever the thermal camera happens to need.
                     'cam_yaw_deg': L('line_cam_yaw_deg'),
                     'flip_lr': L('line_flip_lr'),
                     'flip_ud': L('line_flip_ud'),
                     'blur_ksize': L('line_blur'),
                     'close_ksize': L('line_close_ksize'),
                     'open_ksize': L('line_open_ksize'),
                     'strip_is_bright': L('line_strip_is_bright'),
                     'min_area_frac': L('line_min_area_frac'),
                     'max_area_frac': L('line_max_area_frac'),
                     'min_elongation': L('line_min_elongation'),
                     'downscale': L('line_downscale'),
                     'min_quality': L('line_min_quality'),
                     'stream_port': L('line_stream_port')}],
        condition=IfCondition(L('line')),
    )

    # The end of the corridor, off the ZED's depth image. Needs the ZED
    # wrapper running -- this launch file does not start it.
    wall_node = Node(
        package='drone_testing', executable='wall_watch', name='wall_watch',
        output='screen', emulate_tty=True,
        parameters=[{'depth_topic': L('wall_depth_topic'),
                     'camera_info_topic': L('wall_camera_info_topic'),
                     'stop_distance': L('wall_stop_distance'),
                     'min_pixels': L('wall_min_pixels'),
                     'floor_margin': L('wall_floor_margin'),
                     'camera_pitch_deg': L('wall_camera_pitch_deg')}],
        condition=IfCondition(L('wall')),
    )

    led_node = Node(
        package='drone_testing', executable='led_status', name='led_status',
        output='screen', emulate_tty=True,
        parameters=[{'num_pixels': L('num_pixels'), 'blink_hz': L('blink_hz')}],
        condition=IfCondition(L('led')),
    )

    # Arguments that belong to a NODE OTHER than the flight node, and so must
    # not be forwarded to it: a flight node given an undeclared parameter
    # simply ignores it, which hides a typo until the day it matters.
    not_flight = {
        'agent', 'agent_only', 'refresh_hz', 'publish_preview',
        'flight_node_delay',
        'led', 'num_pixels', 'blink_hz', 'stream_port', 'jpeg_quality',
        'fsm', 'thermal_sim', 'thermal_raw_topic', 'thermal_resolution',
        'thermal_noise_c', 'aruco', 'aruco_camera_index', 'aruco_image_topic',
        'aruco_marker_ids', 'aruco_marker_size', 'aruco_hfov_deg',
        'aruco_dict', 'aruco_stream_port', 'aruco_min_marker_distance_rate',
        'servo_node', 'servo_close_on_start',
        'servo_close_on_arm', 'servo_close_on_arm_seconds',
        'line', 'line_stream_port', 'line_blur', 'line_downscale',
        'line_cam_yaw_deg', 'line_flip_lr', 'line_flip_ud',
        'line_close_ksize', 'line_open_ksize', 'line_strip_is_bright',
        'line_min_area_frac', 'line_max_area_frac', 'line_min_elongation',
        'aruco_publish_frames',
        'wall', 'wall_depth_topic', 'wall_camera_info_topic',
        'wall_min_pixels', 'wall_floor_margin', 'wall_camera_pitch_deg',
    }
    # ...and the ones only thermal_fsm declares. Passed to thermal_drop they
    # would be rejected outright, so the two nodes get two parameter sets.
    fsm_only = {
        'mark_search', 'mark_required', 'mark_search_distance',
        'mark_search_speed', 'mark_min_travel', 'mark_hover_seconds',
        'box_offset_right', 'mark_stage_timeout', 'leg_lookahead',
        'leg_stall_seconds', 'leg_trim_deg', 'leg_bearing_deg',
        'precision_land',
        'land_search_distance', 'land_search_speed', 'land_return',
        'land_return_distance', 'land_return_speed', 'land_return_min_travel',
        'pad_centre_tolerance',
        'pad_centre_seconds', 'pad_descent_rate', 'pad_handoff_height',
        'pad_gain', 'pad_max_nudge', 'pad_lost_seconds', 'pad_stage_timeout',
        'marker_anchor_gain', 'marker_anchor_max_age', 'ground_effect_height',
        'line_follow', 'line_gain', 'line_max_nudge', 'line_lost_seconds',
        'line_start_after',
        'line_min_quality', 'line_max_heading_deg',
        'wall_stop', 'wall_stop_distance', 'wall_lost_seconds',
    }

    common = {n: L(n) for n, _, _ in args if n not in not_flight | fsm_only}
    # Takeoff goes to the CRUISE height. thermal_drop.py overrides
    # TAKEOFF_ALTITUDE with cruise_altitude anyway; this keeps the parameter
    # the node reports consistent with what it actually flies.
    common['takeoff_altitude'] = L('cruise_altitude')
    # The flight node still decides WHEN and still publishes the trigger; it
    # stops sending actuator commands of its own whenever the release node is
    # the one holding the output.
    common['release_via_servo_node'] = L('servo_node')
    fsm_params = dict(common, **{n: L(n) for n, _, _ in args if n in fsm_only})

    # Exactly one of these runs. thermal_fsm IS a thermal_drop, so everything
    # the plain node reads it reads too; it just declares more.
    flight = TimerAction(
        period=L('flight_node_delay'),
        actions=[Node(
            package='drone_testing', executable='thermal_drop', name='thermal_drop',
            output='screen', emulate_tty=True, parameters=[common],
            condition=UnlessCondition(L('fsm')),
        ), Node(
            package='drone_testing', executable='thermal_fsm', name='thermal_fsm',
            output='screen', emulate_tty=True, parameters=[fsm_params],
            condition=IfCondition(L('fsm')),
        )],
        condition=UnlessCondition(L('agent_only')),
    )

    return LaunchDescription(declared + [microxrce, sensor, sensor_sim,
                                         aruco_node, line_node, wall_node,
                                         led_node, servo_node, flight])
