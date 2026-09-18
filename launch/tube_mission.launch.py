"""
The obstacle-course TUBES, flown as their own mission. ARK Flow localisation.

    uXRCE-DDS agent + zed_wrapper (CAMERA ONLY)
    + tube_detect (the uprights) + tube_cross (the flight)

    arm -> climb -> hold -> find the uprights -> fit the three-tube layout ->
    line up on the left gap -> through it -> step left -> on past the back
    tube -> land.

Read window_traverse.launch.py and bar_mission.launch.py first: the
localisation stack, PX4 parameters and CPU budget notes apply unchanged.

BEFORE THE FIRST FLIGHT

1. BENCH THE DETECTOR. Hold the airframe at about 1 m, 2.5-3 m in front of the
   obstacle:

        ros2 launch drone_testing tube_mission.launch.py flight:=false
        ros2 topic echo /tube_info
        # or http://<jetson>:8082/

   You want exactly the three front uprights boxed (plus the back one), NOT
   the diagonal or the cross tube, and depths that match a tape measure. If
   the diagonal survives, raise vertical_kernel_frac; if uprights break up
   where the cross tube meets them, lower it.

2. BENCH THE FIT. Run the flight node unarmed and watch /tube_gap. It should
   read the gap centre in NED, a width close to 0.50, "3 uprights matched",
   and NOT move as you walk the airframe around.

3. PUT IT DOWN POINTING AT THE OBSTACLE, 2.5-3 m back. The yaw cone is 30 deg.

WHAT TO RUN

Bench, no props:

    ros2 launch drone_testing tube_mission.launch.py flight:=false

Flight, keeping the q/k keyboard aborts (second pane):

    ros2 launch drone_testing tube_mission.launch.py
    ros2 run drone_testing tube_cross --ros-args -p cam_x:=0.105 -p cam_z:=-0.04

Everything in one shot (RC kill switch still works):

    ros2 launch drone_testing tube_mission.launch.py agent_only:=false
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CROSS_PARAMS = (
    # the climb (OffboardSequence)
    'takeoff_altitude', 'hold_seconds', 'ground_wait_seconds', 'climb_speed',
    'land_speed', 'yaw_rate', 'takeoff_return_to_pad', 'min_altitude',
    'max_altitude', 'yaw_cone_deg', 'flight_seconds', 'request_offboard_from_ros',
    # the obstacle
    'tube_spacing', 'tube_radius', 'cross_bar_height', 'diagonal_left_height',
    'diagonal_right_height', 'gap_side',
    # the path
    'standoff_distance', 'pass_exit_distance', 'shift_left', 'exit_distance',
    'clearance', 'cross_altitude', 'approach_speed', 'pass_speed', 'shift_speed',
    'align_cross_tolerance', 'align_along_tolerance', 'align_yaw_tolerance_deg',
    'alt_tolerance', 'settle_seconds', 'refine_min_distance', 'refine_max_jump',
    'search_timeout', 'lock_seconds', 'assume_gap_distance', 'assume_gap_left',
    # the airframe
    'gear_below_camera', 'drone_height', 'drone_width',
    # the estimate
    'depth_min', 'depth_max', 'min_tube_top_height', 'max_tube_bottom_height',
    'buffer_seconds', 'cluster_radius', 'pose_min_samples', 'plane_band',
    'match_tolerance', 'min_matched_tubes', 'max_plane_yaw_deg',
)

DETECT_PARAMS = (
    'image_topic', 'depth_topic', 'camera_info_topic', 'color', 'min_area',
    'min_aspect', 'max_tilt_deg', 'vertical_kernel_frac', 'samples_along',
    'border_margin', 'min_tubes', 'detect_frames', 'lost_frames', 'max_fps',
    'fallback_hfov_deg', 'publish_image', 'publish_mask', 'publish_compressed',
    'stream_port', 'stream_scale', 'jpeg_quality',
)

CAMERA_MOUNTING = ('cam_x', 'cam_y', 'cam_z', 'cam_roll', 'cam_pitch', 'cam_yaw')


def _params(names):
    return {name: LaunchConfiguration(name) for name in names}


def generate_launch_description():
    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
    )

    zed_wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('zed')),
    )

    detect_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='drone_testing',
            executable='tube_detect',
            name='tube_detect',
            output='screen',
            emulate_tty=True,
            parameters=[_params(DETECT_PARAMS)],
        )],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    reboot_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='drone_testing',
            executable='fc_reboot',
            name='fc_reboot',
            output='screen',
            emulate_tty=True,
        )],
        condition=IfCondition(LaunchConfiguration('reboot_fc')),
    )

    cross_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[Node(
            package='drone_testing',
            executable='tube_cross',
            name='tube_cross',
            output='screen',
            emulate_tty=True,
            parameters=[dict(_params(CAMERA_MOUNTING), **_params(CROSS_PARAMS))],
        )],
        condition=UnlessCondition(LaunchConfiguration('agent_only')),
    )

    lcd_node = Node(
        package='drone_testing',
        executable='lcd_status',
        name='lcd_status',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': LaunchConfiguration('lcd_port')}],
        condition=IfCondition(LaunchConfiguration('lcd')),
    )

    arg = DeclareLaunchArgument
    return LaunchDescription([
        arg('agent_only', default_value='true',
            description='Start everything but the flight node, so it can be run '
                        'by hand with the q/k keyboard aborts.'),
        arg('flight', default_value='true',
            description='false = camera side only (bench test).'),
        arg('detect', default_value='true'),
        arg('zed', default_value='true'),

        # ---- camera ----
        arg('camera_model', default_value='zed'),
        arg('camera_name', default_value='zed'),
        arg('image_topic', default_value='/zed/zed_node/rgb/color/rect/image'),
        arg('depth_topic', default_value='/zed/zed_node/depth/depth_registered'),
        arg('camera_info_topic', default_value='auto'),
        arg('fallback_hfov_deg', default_value='90.0'),

        # ---- camera mounting (same numbers as window_traverse / bar_mission) ----
        arg('cam_x', default_value='0.105', description='m FORWARD of the CoG.'),
        arg('cam_y', default_value='0.0', description='m LEFT of the CoG.'),
        arg('cam_z', default_value='-0.04', description='m ABOVE the CoG.'),
        arg('cam_roll', default_value='0.0'),
        arg('cam_pitch', default_value='0.0', description='rad, + = pointed DOWN.'),
        arg('cam_yaw', default_value='0.0'),

        # ---- the detector ----
        arg('color', default_value='red'),
        arg('min_area', default_value='600.0'),
        arg('min_aspect', default_value='4.0'),
        arg('max_tilt_deg', default_value='15.0',
            description='How far an upright may lean off vertical in the image.'),
        arg('vertical_kernel_frac', default_value='0.10',
            description='Height of the vertical opening kernel as a fraction '
                        'of the image. Removes the cross tube and the diagonal '
                        'so the uprights come apart into separate contours.'),
        arg('samples_along', default_value='9'),
        arg('border_margin', default_value='8.0'),
        arg('min_tubes', default_value='2'),
        arg('detect_frames', default_value='3'),
        arg('lost_frames', default_value='5'),
        arg('max_fps', default_value='10.0'),
        arg('publish_image', default_value='true'),
        arg('publish_mask', default_value='false'),
        arg('publish_compressed', default_value='true'),
        arg('stream_port', default_value='8082',
            description='8080 window_detect, 8081 bar_detect.'),
        arg('stream_scale', default_value='0.5'),
        arg('jpeg_quality', default_value='60'),

        # ---- the climb ----
        arg('takeoff_altitude', default_value='1.0',
            description='About the gap altitude, so the uprights are square '
                        'in the image during SEARCH.'),
        arg('hold_seconds', default_value='5.0'),
        arg('ground_wait_seconds', default_value='5.0'),
        arg('climb_speed', default_value='0.35'),
        arg('land_speed', default_value='0.15'),
        arg('yaw_rate', default_value='0.35'),
        arg('takeoff_return_to_pad', default_value='false'),
        arg('min_altitude', default_value='0.4'),
        arg('max_altitude', default_value='2.0'),
        arg('yaw_cone_deg', default_value='30.0'),
        arg('flight_seconds', default_value='150.0'),
        arg('request_offboard_from_ros', default_value='true'),

        # ---- the obstacle (rules drawing) ----
        arg('tube_spacing', default_value='0.50'),
        arg('tube_radius', default_value='0.025',
            description='m. MEASURE THE REAL TUBE. It comes straight out of '
                        'the lateral margin.'),
        arg('cross_bar_height', default_value='0.461'),
        arg('diagonal_left_height', default_value='2.0'),
        arg('diagonal_right_height', default_value='0.922'),
        arg('gap_side', default_value='left',
            description='left = between the left and middle uprights (the big '
                        'gap). "Left" is as seen by the aircraft approaching.'),

        # ---- the path ----
        arg('standoff_distance', default_value='1.20'),
        arg('pass_exit_distance', default_value='0.50',
            description='m past the tube plane before the sideways step.'),
        arg('shift_left', default_value='0.40',
            description='m sideways after the gap, + = LEFT. Clears the back '
                        'upright.'),
        arg('exit_distance', default_value='1.20',
            description='m on from the shift point. The back upright is 1 m '
                        'behind the plane.'),
        arg('clearance', default_value='0.12',
            description='m vertical clearance to the cross tube below and the '
                        'diagonal above.'),
        arg('cross_altitude', default_value='0.0',
            description='0 = solve it from the geometry (about 1.06 m). '
                        'Refused if outside the band that fits.'),
        arg('approach_speed', default_value='0.30'),
        arg('pass_speed', default_value='0.30'),
        arg('shift_speed', default_value='0.25'),
        arg('align_cross_tolerance', default_value='0.05',
            description='m off the gap centreline before committing. Under '
                        '10 cm a side in the gap, so keep this tight.'),
        arg('align_along_tolerance', default_value='0.15'),
        arg('align_yaw_tolerance_deg', default_value='5.0'),
        arg('alt_tolerance', default_value='0.06'),
        arg('settle_seconds', default_value='1.5'),
        arg('refine_min_distance', default_value='1.0'),
        arg('refine_max_jump', default_value='0.20'),
        arg('search_timeout', default_value='45.0'),
        arg('lock_seconds', default_value='2.0'),
        arg('assume_gap_distance', default_value='0.0',
            description='> 0 = FLY BLIND: gap assumed this far ahead of the '
                        'aircraft after the hold. Not recommended here.'),
        arg('assume_gap_left', default_value='0.0'),

        # ---- the airframe ----
        arg('gear_below_camera', default_value='0.120'),
        arg('drone_height', default_value='0.260'),
        arg('drone_width', default_value='0.260',
            description='Prop tip to prop tip.'),

        # ---- the estimate ----
        arg('depth_min', default_value='0.40'),
        arg('depth_max', default_value='6.0'),
        arg('min_tube_top_height', default_value='1.20'),
        arg('max_tube_bottom_height', default_value='0.40'),
        arg('buffer_seconds', default_value='2.5'),
        arg('cluster_radius', default_value='0.15'),
        arg('pose_min_samples', default_value='6'),
        arg('plane_band', default_value='0.40',
            description='m behind the nearest upright still counted as the '
                        'front plane. Keeps the back upright (1 m) out.'),
        arg('match_tolerance', default_value='0.12'),
        arg('min_matched_tubes', default_value='3',
            description='3 = all front uprights must be seen. With 2 the '
                        'middle tube is ambiguous.'),
        arg('max_plane_yaw_deg', default_value='30.0'),

        # ---- the rest ----
        arg('reboot_fc', default_value='false'),
        arg('flight_node_delay', default_value='12.0'),
        arg('lcd', default_value='false'),
        arg('lcd_port', default_value=''),

        GroupAction([microxrce_node, lcd_node, reboot_node, cross_node],
                    condition=IfCondition(LaunchConfiguration('flight'))),
        zed_wrapper,
        detect_node,
    ])
