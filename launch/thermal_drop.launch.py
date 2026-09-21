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

THE WHOLE MISSION, NOT JUST THE DROP

    fsm:=true swaps the flight node for thermal_fsm, which flies

        takeoff -> forward to an ArUco marker -> hover 1 s on it
        -> one step RIGHT onto the boxes -> the survey, the drop and the
        retreat below -> forward to the LANDING marker -> precision landing

    Everything in this file still applies: thermal_fsm IS a thermal_drop
    with two ends bolted on, and reads every parameter here. It needs a
    downward camera and the ArUco detector, so:

        ros2 launch drone_testing thermal_drop.launch.py fsm:=true \\
            aruco:=true aruco_image_topic:=/camera/down/image_raw \\
            aruco_min_marker_distance_rate:=0.02

    In the simulator all of that is set for you:

        ros2 launch drone_testing thermal_drop_sitl.launch.py

    which also swaps thermal_sensor (MLX90640 over I2C) for thermal_sim
    (the Gazebo thermal camera) and scales every length by 2.2. Read that
    file's header for the arena, the scaling rule and the flight plan.

THE SERVO

    Test it on its own first -- the mission is a slow way to debug a servo:

        ros2 run drone_testing servo_test --ros-args -p command:=sweep

    That sends MAV_CMD_ACTUATOR_TEST across Servo 1-8 (functions 201-208),
    one a second, and prints PX4's ack for each. Watch the servo, note the
    function that moves it, then:

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
                              number, so set servo_function too.

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
        ('search_radius', '1.5', 'm from the survey centre; blobs further out ignored.'),
        ('cluster_radius', '0.30', ''),
        ('min_hot_margin', '2.0', 'deg C the hottest box should lead by (warning only).'),
        ('verify_frames', '8', 'Frames of evidence before the descent may start.'),
        ('verify_window', '3.0', 's the hottest-in-frame evidence is counted over.'),
        ('verify_ratio', '0.7', 'Fraction of those frames the box must be hottest in.'),
        ('retarget_margin', '1.5',
         'deg C another blob must beat the target by, consistently, to steal it.'),

        # ---- flight ----
        ('survey_altitude', '1.5', 'm, clamped to 1.6 in the node.'),
        ('drop_altitude', '0.5', 'm above the floor, floored at 0.5 in the node.'),
        ('survey_dwell_seconds', '4.0', ''),
        ('survey_step', '0.5', 'm, ring of extra survey points if boxes are missing.'),
        ('survey_all_points', 'false', 'true = always fly the whole ring.'),
        ('approach_tolerance', '0.15', ''),
        ('descend_tolerance', '0.12', 'm; descent pauses while further off than this.'),
        ('descend_speed', '0.12', ''),
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
        ('sim_descend_speed', '0.25', 'm/s the SIMULATED vehicle descends at.'),
        ('retreat_altitude', '1.2', 'm climbed back to after the drop.'),
        ('retreat_right', '0.5', 'm stepped to the RIGHT before landing.'),
        ('land_after_drop', 'true', 'false = hold clear of the box instead.'),
        ('track_gate', '0.40', ''),
        ('flight_seconds', '150.0', ''),
        ('hold_seconds', '4.0', ''),
        ('ground_wait_seconds', '5.0', ''),
        ('climb_speed', '0.35', ''),
        ('land_speed', '0.15', ''),
        ('move_speed', '0.25', ''),

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
        ('move_leash', '0.40', 'm the commanded x/y may lead the measured x/y.'),
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
        ('fsm', 'false',
         'true runs thermal_fsm (marker -> boxes -> drop -> marker -> land) '
         'instead of thermal_drop (survey here -> drop -> land).'),

        # ---- the legs thermal_fsm adds. REAL course metres. ----
        ('mark_search', 'true',
         'false skips the marker hunt and surveys from the takeoff point, '
         'which is thermal_drop\'s own mission.'),
        ('mark_required', 'true',
         'true = a marker that is never found ENDS the mission. false = '
         'survey from wherever the creep gave up (normally bare floor).'),
        ('mark_search_distance', '11.0',
         'm of forward creep before the marker hunt gives up.'),
        ('mark_search_speed', '0.30', ''),
        ('mark_min_travel', '1.00',
         'm that must be flown before a marker counts -- the aircraft arms ON '
         'a marked pad and must not "find" the one it is standing on.'),
        ('mark_hover_seconds', '1.0',
         's stationary over the marker. A TIME, not a length: never scaled.'),
        ('box_offset_right', '1.50',
         'm RIGHT of the marker, which is where the boxes are.'),
        ('mark_stage_timeout', '60.0', ''),
        ('leg_lookahead', '0.70',
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
        ('leg_trim_deg', '0.0',
         'deg added to every leg heading, +ve to the RIGHT. For a KNOWN, '
         'repeatable aim bias only. Leave at 0 until a leg has been flown '
         'and measured; the log prints the cross-track error of each one.'),

        # ---- the precision landing after the drop ----
        ('precision_land', 'true',
         'false = plain PX4 land after the retreat, as thermal_drop does.'),
        ('land_search_distance', '2.50',
         'm of forward creep after the retreat, looking for the landing pad.'),
        ('land_search_speed', '0.30', ''),
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
        ('max_survey_altitude', '1.6',
         'm above which the MLX readings stop being usable. A HARD ceiling on '
         'the aircraft; raised only by the scaled simulation, where the whole '
         'arena is 2.2x further away.'),

        # ---- the downward ArUco detector the marker stages fly on ----
        ('aruco', 'false',
         'Start aruco_pose on the downward camera. Required by fsm:=true '
         'unless something else is already publishing /aruco/point.'),
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
        ('aruco_min_marker_distance_rate', '0.05',
         'cv2.aruco merges two candidate quads whose corners are within this '
         'fraction of the image and keeps the LARGER. A marker lying on a '
         'square PAD has the pad\'s outline as a second, concentric '
         'candidate, and at the default 0.05 it swallows the marker and '
         'NOTHING is ever detected. The simulator passes 0.02. 0 is not '
         '"off" -- it detects nothing either.'),

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
                     'stream_port': L('aruco_stream_port')}],
        condition=IfCondition(L('aruco')),
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
    }
    # ...and the ones only thermal_fsm declares. Passed to thermal_drop they
    # would be rejected outright, so the two nodes get two parameter sets.
    fsm_only = {
        'mark_search', 'mark_required', 'mark_search_distance',
        'mark_search_speed', 'mark_min_travel', 'mark_hover_seconds',
        'box_offset_right', 'mark_stage_timeout', 'leg_lookahead',
        'leg_trim_deg', 'leg_bearing_deg', 'precision_land',
        'land_search_distance', 'land_search_speed', 'pad_centre_tolerance',
        'pad_centre_seconds', 'pad_descent_rate', 'pad_handoff_height',
        'pad_gain', 'pad_max_nudge', 'pad_lost_seconds', 'pad_stage_timeout',
    }

    common = {n: L(n) for n, _, _ in args if n not in not_flight | fsm_only}
    common['takeoff_altitude'] = L('survey_altitude')
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
                                         aruco_node, led_node, flight])
