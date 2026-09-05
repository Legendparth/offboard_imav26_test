"""
Window scan launch: uXRCE-DDS agent + ZED + window detection + the flight.

    arm -> climb to takeoff_altitude -> hold -> sweep the nose through a
    90 degree arc until the ZED sees the window -> lock the yaw and the
    position -> land 40 s after the climb started.

WHAT TO RUN

Bench test, no props, camera only -- this is how you check the detection
and the topic names before anything spins:

    ros2 launch drone_testing window_scan.launch.py flight:=false

    ros2 topic echo /window_detected
    ros2 run rqt_image_view rqt_image_view /window_detection/image

Flight. The default starts the agent, the ZED and the detector but NOT the
flight node, so you can run that one by hand and keep the q/k keyboard
aborts (a node started by launch has no tty, so those keys are dead):

    ros2 launch drone_testing window_scan.launch.py
    ros2 run drone_testing window_scan --ros-args \
        -p takeoff_altitude:=1.0 -p flight_seconds:=40.0

Everything in one shot, no keyboard abort (your RC kill switch still
works, and it is the one that matters):

    ros2 launch drone_testing window_scan.launch.py agent_only:=false

If zed_wrapper is already running from another launch file, add
zed:=false so this one does not start a second copy of it.

CHECK THE IMAGE TOPIC FIRST. The default is the standard zed_wrapper name
for the rectified left colour image:

    ros2 topic list | grep zed

and if yours differs, pass image_topic:=/your/topic (and depth_topic:=...).
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')
    flight = LaunchConfiguration('flight')

    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
    )

    # The camera driver. Skipped with zed:=false if you already have one up.
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('zed_wrapper'), 'launch', 'zed_camera.launch.py'])),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
            'camera_name': LaunchConfiguration('camera_name'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('zed')),
    )

    # Detection. Delayed a little: the ZED SDK takes a few seconds to open the
    # camera, and starting the detector into a topic that does not exist yet
    # just fills the log with "no frames" before the first frame arrives.
    detect_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='window_detect',
                name='window_detect',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic'),
                    'depth_topic': LaunchConfiguration('depth_topic'),
                    'show_windows': LaunchConfiguration('show_windows'),
                    'publish_image': LaunchConfiguration('publish_image'),
                    'publish_mask': LaunchConfiguration('publish_mask'),
                    'color': LaunchConfiguration('color'),
                    'min_area': LaunchConfiguration('min_area'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    # The flight. Held back until the DDS session is up and PX4's topics
    # exist, or the first setpoints are dropped.
    scan_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='drone_testing',
                executable='window_scan',
                name='window_scan',
                output='screen',
                emulate_tty=True,
                parameters=[{
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                    'relock_on_loss': LaunchConfiguration('relock_on_loss'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                }],
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    # Status on the Arduino TFT. Started with the agent so the screen is alive
    # from boot; it also shows the window detection on row 4 by itself.
    lcd_node = Node(
        package='drone_testing',
        executable='lcd_status',
        name='lcd_status',
        output='screen',
        emulate_tty=True,
        parameters=[{'port': LaunchConfiguration('lcd_port')}],
        condition=IfCondition(LaunchConfiguration('lcd')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start the agent, the ZED and the detector but not the '
                        'flight node, so you can run that by hand and keep the '
                        'q/k keyboard aborts. false = fly the whole thing.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera and detection only: no DDS agent, no '
                        'flight node. Use this for the bench test.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start the window_detect node.'),
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='Start zed_wrapper. false if it is already running.'),
        DeclareLaunchArgument(
            'camera_model', default_value='zed',
            description='ZED model passed to zed_wrapper (zed, zedm, zed2, ...).'),
        DeclareLaunchArgument(
            'camera_name', default_value='zed',
            description='Namespace zed_wrapper publishes under; the default '
                        'image_topic below assumes "zed".'),
        DeclareLaunchArgument(
            'image_topic', default_value='/zed/zed_node/rgb/image_rect_color',
            description='Rectified colour image from the left camera. Check '
                        'yours with `ros2 topic list | grep zed`.'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/zed/zed_node/depth/depth_registered',
            description='Depth map registered to image_topic, 32FC1 in metres.'),
        DeclareLaunchArgument(
            'show_windows', default_value='false',
            description='cv2.imshow the frame and the mask. Needs a display; '
                        'leave false on a headless Jetson.'),
        DeclareLaunchArgument(
            'publish_image', default_value='true',
            description='Publish the annotated frame on /window_detection/image.'),
        DeclareLaunchArgument(
            'publish_mask', default_value='false',
            description='Also publish the HSV mask, for tuning the thresholds.'),
        DeclareLaunchArgument(
            'color', default_value='green',
            description='Which HSV range to look for: green, blue or red.'),
        DeclareLaunchArgument(
            'min_area', default_value='1500.0',
            description='px^2 the contour must exceed to count as a window.'),
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.0',
            description='Metres above the arming point. Start lower (0.5) on '
                        'the first flights.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='40.0',
            description='Seconds from the START OF THE CLIMB to the descent, '
                        'window found or not.'),
        DeclareLaunchArgument(
            'scan_span_deg', default_value='90.0',
            description='Total width of the yaw sweep, centred on the takeoff '
                        'heading: 90 = 45 deg either side.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s the yaw setpoint is walked at (~20 deg/s). Keep '
                        'it slow: a fast yaw smears the optical flow the '
                        'position hold depends on, and blurs the camera.'),
        DeclareLaunchArgument(
            'detect_seconds', default_value='0.4',
            description='How long /window_detected must stay true before the '
                        'sweep stops. On top of the detector own debounce.'),
        DeclareLaunchArgument(
            'relock_on_loss', default_value='false',
            description='true = go back to sweeping if the window is lost '
                        'after the lock. false = stay put.'),
        DeclareLaunchArgument(
            'hold_seconds', default_value='5.0',
            description='Station keeping at altitude before the sweep starts. '
                        'The optical flow x/y latch has to happen in here.'),
        DeclareLaunchArgument(
            'ground_wait_seconds', default_value='5.0',
            description='Time armed on the ground before the climb.'),
        DeclareLaunchArgument(
            'climb_speed', default_value='0.35',
            description='m/s the climb setpoint is ramped at.'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='m/s the descent setpoint is ramped at. PX4 only counts '
                        'it as a descent if it is at least 0.9 * MPC_LAND_SPEED, '
                        'so set MPC_LAND_SPEED to about 0.2 to match.'),
        DeclareLaunchArgument(
            'min_altitude', default_value='0.4',
            description='m above the arming point the flight may not go below.'),
        DeclareLaunchArgument(
            'max_altitude', default_value='3.0',
            description='m above the arming point the flight may not exceed.'),
        DeclareLaunchArgument(
            'request_offboard_from_ros', default_value='true',
            description='false = you flip the Offboard switch on the TX.'),
        DeclareLaunchArgument(
            'lcd', default_value='true',
            description='Start the Arduino TFT status node.'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect ttyACM*/ttyUSB*.'),

        # flight:=false leaves just the camera side running, which is what you
        # want when you are tuning HSV thresholds indoors. The PX4-facing
        # actions are grouped rather than given a condition directly, because
        # two of them already carry one of their own and an action's condition
        # is fixed when it is built.
        GroupAction([microxrce_node, lcd_node, scan_node],
                    condition=IfCondition(flight)),
        zed_launch,
        detect_node,
    ])
