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

3. Everything in one shot (RC kill switch still works, keyboard does not):

       ros2 launch drone_testing thermal_drop.launch.py agent_only:=false

Watch:  ros2 topic echo /thermal/hotspot      (sensor alive?)
        ros2 topic echo /thermal_drop/target  (box_n|box_e|err|stage)
        ros2 topic echo /thermal_drop/ready   (true = aligned at drop height)
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
        ('mode', 'fly', 'bench = log only, nothing sent to PX4. fly = the mission.'),

        # ---- sensor ----
        ('refresh_hz', '8', 'MLX90640 refresh: 1/2/4/8/16. 8 is the useful max on I2C.'),
        ('publish_preview', 'false', 'Colourised thermal/preview image for rqt.'),
        ('hfov_deg', '110.0', ''),
        ('vfov_deg', '75.0', ''),
        ('cam_yaw_deg', '0.0', 'Image-top rotation from the nose, about the down axis.'),
        ('flip_lr', 'false', ''),
        ('flip_ud', 'false', ''),
        ('min_contrast', '3.0', 'deg C above the frame median to count as a blob.'),

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
        ('hover_seconds', '10.0', ''),
        ('land_after_hover', 'true', ''),
        ('track_gate', '0.40', ''),
        ('flight_seconds', '150.0', ''),
        ('hold_seconds', '4.0', ''),
        ('ground_wait_seconds', '5.0', ''),
        ('climb_speed', '0.35', ''),
        ('land_speed', '0.15', ''),
        ('move_speed', '0.25', ''),

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
                     'publish_preview': L('publish_preview')}],
    )

    flight_params = {n: L(n) for n, _, _ in args
                     if n not in ('agent_only', 'refresh_hz', 'publish_preview',
                                  'flight_node_delay')}
    flight_params['takeoff_altitude'] = L('survey_altitude')

    flight = TimerAction(
        period=L('flight_node_delay'),
        actions=[Node(
            package='drone_testing', executable='thermal_drop', name='thermal_drop',
            output='screen', emulate_tty=True, parameters=[flight_params],
        )],
        condition=UnlessCondition(L('agent_only')),
    )

    return LaunchDescription(declared + [microxrce, sensor, flight])
