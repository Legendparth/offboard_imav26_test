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

    args = [
        ('agent_only', 'true', 'Start agent + sensor but not the flight node.'),
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
    ]
    declared = [DeclareLaunchArgument(n, default_value=d, description=h)
                for n, d, h in args]

    microxrce = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_xrce_dds_agent', output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
        condition=UnlessCondition(bench),
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
    )

    led_node = Node(
        package='drone_testing', executable='led_status', name='led_status',
        output='screen', emulate_tty=True,
        parameters=[{'num_pixels': L('num_pixels'), 'blink_hz': L('blink_hz')}],
        condition=IfCondition(L('led')),
    )

    flight_params = {n: L(n) for n, _, _ in args
                     if n not in ('agent_only', 'refresh_hz', 'publish_preview',
                                  'flight_node_delay', 'led', 'num_pixels',
                                  'blink_hz', 'stream_port', 'jpeg_quality')}
    flight_params['takeoff_altitude'] = L('survey_altitude')

    flight = TimerAction(
        period=L('flight_node_delay'),
        actions=[Node(
            package='drone_testing', executable='thermal_drop', name='thermal_drop',
            output='screen', emulate_tty=True, parameters=[flight_params],
        )],
        condition=UnlessCondition(L('agent_only')),
    )

    return LaunchDescription(declared + [microxrce, sensor, led_node, flight])
