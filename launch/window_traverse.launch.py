"""
Window traversal on ZED visual odometry.

    uXRCE-DDS agent + zed_wrapper + zed_localization (VIO into PX4)
    + window_detect (detection AND geometry) + window_traverse (the flight)

    arm -> climb -> hold -> sweep for the window -> lock and build a pose
    estimate -> turn onto the window normal -> fly to a point in front of it
    -> fly through it -> hold on the far side -> land.

This is window_scan.launch.py's mission continued past the lock, flown on
the localisation stack from sequence_vio_test.launch.py. Read BOTH of those
files first -- everything they say about PX4 parameters, about the camera
having no IMU, and about MPC_LAND_SPEED applies here unchanged and is not
repeated.

WHAT TO RUN

Bench, no props: the whole camera side, no agent and no flight node. This is
where you check that the geometry topic is alive and sane before anything
spins, and you can do it holding the airframe in your hands:

    ros2 launch drone_testing window_traverse.launch.py flight:=false

    ros2 topic echo /window_detected
    ros2 topic echo /window_geometry      # 15 floats: 5 points x (d, az, el)
    ros2 run rqt_image_view rqt_image_view /window_detection/image

Flight. The default starts everything EXCEPT the flight node, so you run
that in a second pane and keep the q/k keyboard aborts (a node started by
launch has no tty, so those keys are dead):

    ros2 launch drone_testing window_traverse.launch.py
    ros2 run drone_testing window_traverse --ros-args \
        -p takeoff_altitude:=1.2 -p standoff_distance:=1.6 \
        -p cam_pitch:=0.0 -p cam_x:=0.10

    ros2 topic echo /window_pose          # x|y|z|yaw|w|h|samples|age, NED

Everything in one shot, no keyboard abort (the RC kill switch still works,
and it is the one that matters):

    ros2 launch drone_testing window_traverse.launch.py agent_only:=false

BEFORE THE FIRST FLIGHT

1. THE CAMERA MOUNTING NUMBERS ARE PASSED TO TWO NODES AND MUST MATCH.
   cam_x/cam_y/cam_z and cam_roll/cam_pitch/cam_yaw below go both to
   zed_localization (which uses them to turn the camera's odometry into the
   vehicle's) and to window_traverse (which uses them to turn a pixel into a
   point in front of the vehicle). This launch file feeds one set of
   arguments to both so they cannot disagree. If you run either node by hand,
   pass the same numbers. They are the pose of the camera in the body frame
   in the ROS convention: x forward, y LEFT, z UP, radians, and the defaults
   ("at the CoG, pointing straight forward") are almost certainly not yours.

2. CHECK cs_yaw_align. Exactly as in sequence_vio_test.launch.py: with the
   magnetometer off you need EKF2_EV_CTRL=9, EKF2_MAG_TYPE=5 and
   pose_frame:=ned, or PX4 takes the aircraft a second after arming.

        ros2 topic echo /fmu/out/estimator_status_flags --once | grep cs_yaw_align

3. WALK THE ESTIMATE BEFORE YOU FLY IT. With the props off, carry the
   airframe to where you expect it to hover and watch /window_pose. The
   centre should sit still in NED to within a few centimetres while you move
   the airframe around -- that is the whole point of the estimate being in
   NED rather than in the camera frame, and it is the one test that catches a
   wrong cam_pitch or a wrong pose_frame before it costs you an airframe.

4. MEASURE THE WINDOW. /window_pose reports the width and height the
   estimator reconstructs. If they do not match a tape measure, the
   intrinsics or the depth scale are wrong, and every distance in the
   approach is wrong by the same factor.

GEOMETRY OF THE APPROACH

    standoff_distance   where the aircraft lines up: this far in front of the
                        window plane, ON THE WINDOW'S AXIS, at the window's
                        height, nose along the normal. Bigger is a longer,
                        better-conditioned look at the window; smaller is a
                        shorter run through. 1.6 m keeps a 1 m window fully in
                        frame on a 90 degree lens.
    exit_distance       how far past the window plane the run ends.
    approach_speed      m/s for the line-up.
    traverse_speed      m/s for the run through. The estimate is frozen by
                        then, so this is only limited by how much control
                        authority you want in the aperture.
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

    # The mounting pose of the camera, shared verbatim between the bridge and
    # the flight node. One dict, two consumers: see item 1 in the header.
    camera_mounting = {
        'cam_x': LaunchConfiguration('cam_x'),
        'cam_y': LaunchConfiguration('cam_y'),
        'cam_z': LaunchConfiguration('cam_z'),
        'cam_roll': LaunchConfiguration('cam_roll'),
        'cam_pitch': LaunchConfiguration('cam_pitch'),
        'cam_yaw': LaunchConfiguration('cam_yaw'),
    }

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
            # PX4 is the navigation authority on this vehicle; the ZED must not
            # publish a competing odom -> base_link edge.
            'publish_tf': 'false',
            'publish_map_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('zed')),
    )

    # VIO into PX4. Delayed so its first health report is about a camera that
    # has had a chance to open.
    bridge_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='zed_localization',
                name='zed_localization',
                output='screen',
                emulate_tty=True,
                parameters=[dict(camera_mounting, **{
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'pose_frame': LaunchConfiguration('pose_frame'),
                    'publish_velocity': LaunchConfiguration('publish_velocity'),
                    'publish_rate': LaunchConfiguration('publish_rate'),
                })],
                remappings=[('vio_healthy', '/vio_healthy'),
                            ('vio_status', '/vio_status')],
            )
        ],
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    # Detection and geometry. Same node and the same delay as window_scan; the
    # only addition is camera_info_topic, which is what makes /window_geometry
    # metric rather than a guess from an assumed field of view.
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
                    'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                    'publish_geometry': True,
                    'show_windows': LaunchConfiguration('show_windows'),
                    'publish_image': LaunchConfiguration('publish_image'),
                    'publish_mask': LaunchConfiguration('publish_mask'),
                    'color': LaunchConfiguration('color'),
                    'min_area': LaunchConfiguration('min_area'),
                    'stream_port': LaunchConfiguration('stream_port'),
                    'stream_scale': LaunchConfiguration('stream_scale'),
                    'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                }],
            )
        ],
        condition=IfCondition(LaunchConfiguration('detect')),
    )

    reboot_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drone_testing',
                executable='fc_reboot',
                name='fc_reboot',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=IfCondition(LaunchConfiguration('reboot_fc')),
    )

    traverse_node = TimerAction(
        period=LaunchConfiguration('flight_node_delay'),
        actions=[
            Node(
                package='drone_testing',
                executable='window_traverse',
                name='window_traverse',
                output='screen',
                emulate_tty=True,
                parameters=[dict(camera_mounting, **{
                    # ---- the climb and the sweep (inherited) ----
                    'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
                    'hold_seconds': LaunchConfiguration('hold_seconds'),
                    'ground_wait_seconds': LaunchConfiguration('ground_wait_seconds'),
                    'climb_speed': LaunchConfiguration('climb_speed'),
                    'land_speed': LaunchConfiguration('land_speed'),
                    'yaw_rate': LaunchConfiguration('yaw_rate'),
                    'min_altitude': LaunchConfiguration('min_altitude'),
                    'max_altitude': LaunchConfiguration('max_altitude'),
                    'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                    'detect_seconds': LaunchConfiguration('detect_seconds'),
                    'relock_on_loss': LaunchConfiguration('relock_on_loss'),
                    'flight_seconds': LaunchConfiguration('flight_seconds'),
                    'request_offboard_from_ros': LaunchConfiguration(
                        'request_offboard_from_ros'),
                    'hold_xy_from_ground': LaunchConfiguration('hold_xy_from_ground'),
                    'vio_settle_seconds': LaunchConfiguration('vio_settle_seconds'),
                    # ---- the traversal ----
                    'standoff_distance': LaunchConfiguration('standoff_distance'),
                    'exit_distance': LaunchConfiguration('exit_distance'),
                    'altitude_offset': LaunchConfiguration('altitude_offset'),
                    'approach_speed': LaunchConfiguration('approach_speed'),
                    'traverse_speed': LaunchConfiguration('traverse_speed'),
                    'align_tolerance': LaunchConfiguration('align_tolerance'),
                    'align_yaw_tolerance_deg': LaunchConfiguration(
                        'align_yaw_tolerance_deg'),
                    'align_settle_seconds': LaunchConfiguration('align_settle_seconds'),
                    'align_timeout': LaunchConfiguration('align_timeout'),
                    'traverse_timeout': LaunchConfiguration('traverse_timeout'),
                    'clear_seconds': LaunchConfiguration('clear_seconds'),
                    'blind_traverse_seconds': LaunchConfiguration(
                        'blind_traverse_seconds'),
                    # ---- the estimator ----
                    'depth_min': LaunchConfiguration('depth_min'),
                    'depth_max': LaunchConfiguration('depth_max'),
                    'corner_spread': LaunchConfiguration('corner_spread'),
                    'corner_spread_frac': LaunchConfiguration('corner_spread_frac'),
                    'plane_tolerance': LaunchConfiguration('plane_tolerance'),
                    'window_min_size': LaunchConfiguration('window_min_size'),
                    'window_max_size': LaunchConfiguration('window_max_size'),
                    'max_tilt_deg': LaunchConfiguration('max_tilt_deg'),
                    'buffer_seconds': LaunchConfiguration('buffer_seconds'),
                    'pose_min_samples': LaunchConfiguration('pose_min_samples'),
                    'pose_max_age': LaunchConfiguration('pose_max_age'),
                    'pose_lost_timeout': LaunchConfiguration('pose_lost_timeout'),
                    'gate_metres': LaunchConfiguration('gate_metres'),
                    'gate_yaw_deg': LaunchConfiguration('gate_yaw_deg'),
                })],
            )
        ],
        condition=UnlessCondition(agent_only),
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

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start the support stack but not the flight node, so you '
                        'can run that by hand and keep the q/k keyboard aborts. '
                        'false = fly the whole thing from this launch file.'),
        DeclareLaunchArgument(
            'flight', default_value='true',
            description='false = camera side only: no DDS agent, no VIO bridge, '
                        'no flight node. This is the bench test.'),
        DeclareLaunchArgument(
            'detect', default_value='true',
            description='Start window_detect.'),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Start zed_localization, the VIO bridge into PX4.'),
        DeclareLaunchArgument(
            'zed', default_value='true',
            description='Start zed_wrapper. false if it is already running.'),

        # ---- camera ----
        DeclareLaunchArgument('camera_model', default_value='zed'),
        DeclareLaunchArgument('camera_name', default_value='zed'),
        DeclareLaunchArgument(
            'image_topic', default_value='/zed/zed_node/rgb/color/rect/image',
            description='Rectified colour image from the left camera.'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/zed/zed_node/depth/depth_registered',
            description='Depth registered to image_topic, 32FC1 in metres.'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/zed/zed_node/rgb/camera_info',
            description='CameraInfo for image_topic. This is where the geometry '
                        'gets its intrinsics; a wrong topic here means the node '
                        'falls back to a guessed field of view and every angle '
                        'is scaled wrong. It says so in the log if it does.'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/zed/zed_node/odom',
            description='ZED odometry the bridge converts into PX4 vision.'),
        DeclareLaunchArgument(
            'pose_frame', default_value='frd',
            description='frd with the magnetometer ON; ned with it OFF. Read the '
                        'frame table in zed_localization.py -- getting this wrong '
                        'is the "PX4 takes the aircraft a second after arming" bug.'),
        DeclareLaunchArgument('publish_rate', default_value='15.0'),
        DeclareLaunchArgument('publish_velocity', default_value='false'),
        DeclareLaunchArgument('show_windows', default_value='false'),
        DeclareLaunchArgument('publish_image', default_value='true'),
        DeclareLaunchArgument('publish_mask', default_value='false'),
        DeclareLaunchArgument(
            'color', default_value='green',
            description='HSV range to look for: green, blue or red.'),
        DeclareLaunchArgument('min_area', default_value='1500.0'),
        DeclareLaunchArgument('stream_port', default_value='8080'),
        DeclareLaunchArgument('stream_scale', default_value='0.5'),
        DeclareLaunchArgument('jpeg_quality', default_value='60'),

        # ---- camera mounting: pose of the camera IN THE BODY FRAME, ROS
        # convention (x fwd, y LEFT, z UP), metres and radians. Fed to BOTH the
        # bridge and the flight node -- see item 1 in the header. MEASURE THESE.
        DeclareLaunchArgument(
            'cam_x', default_value='0.0',
            description='Metres the camera sits FORWARD of the CoG.'),
        DeclareLaunchArgument(
            'cam_y', default_value='0.0',
            description='Metres the camera sits to the LEFT of the CoG.'),
        DeclareLaunchArgument(
            'cam_z', default_value='0.0',
            description='Metres the camera sits ABOVE the CoG.'),
        DeclareLaunchArgument(
            'cam_roll', default_value='0.0',
            description='Radians, positive = right side down.'),
        DeclareLaunchArgument(
            'cam_pitch', default_value='0.0',
            description='Radians, POSITIVE = camera pointed DOWN. A window is '
                        'looked at level, so this is usually 0 here even if the '
                        'airframe carries the camera tilted for landing.'),
        DeclareLaunchArgument(
            'cam_yaw', default_value='0.0',
            description='Radians, positive = camera pointed to the LEFT.'),

        # ---- the climb and the sweep ----
        DeclareLaunchArgument(
            'takeoff_altitude', default_value='1.2',
            description='m above the arming point. This only has to get the '
                        'camera looking at the window; the traverse then flies '
                        'at the window centre height, whatever that turns out '
                        'to be, clamped into min/max_altitude.'),
        DeclareLaunchArgument('hold_seconds', default_value='5.0'),
        DeclareLaunchArgument('ground_wait_seconds', default_value='5.0'),
        DeclareLaunchArgument('climb_speed', default_value='0.35'),
        DeclareLaunchArgument(
            'land_speed', default_value='0.15',
            description='Keep MPC_LAND_SPEED at about 0.2 so PX4 agrees this is '
                        'a descent.'),
        DeclareLaunchArgument(
            'yaw_rate', default_value='0.35',
            description='rad/s the yaw setpoint is walked at, for the sweep and '
                        'for the turn onto the window normal. Keep it slow: a '
                        'fast yaw is the most reliable way to make an IMU-less '
                        'stereo camera lose tracking.'),
        DeclareLaunchArgument('min_altitude', default_value='0.4'),
        DeclareLaunchArgument('max_altitude', default_value='3.0'),
        DeclareLaunchArgument(
            'scan_span_deg', default_value='90.0',
            description='Total width of the yaw sweep about the takeoff heading.'),
        DeclareLaunchArgument('detect_seconds', default_value='0.4'),
        DeclareLaunchArgument(
            'relock_on_loss', default_value='false',
            description='true = go back to sweeping if the detection is lost '
                        'while still locked. Has no effect once the approach '
                        'has started.'),
        DeclareLaunchArgument(
            'flight_seconds', default_value='150.0',
            description='Hard limit from the START OF THE CLIMB to the descent. '
                        'Fires from every stage EXCEPT the traverse itself -- '
                        'the aircraft is never landed from inside a window.'),
        DeclareLaunchArgument('request_offboard_from_ros', default_value='true'),
        DeclareLaunchArgument('hold_xy_from_ground', default_value='true'),
        DeclareLaunchArgument('vio_settle_seconds', default_value='2.0'),

        # ---- the traversal ----
        DeclareLaunchArgument(
            'standoff_distance', default_value='1.6',
            description='m in front of the window plane the approach lines up '
                        'on, along the window normal.'),
        DeclareLaunchArgument(
            'exit_distance', default_value='1.5',
            description='m beyond the window plane the run through ends.'),
        DeclareLaunchArgument(
            'altitude_offset', default_value='0.0',
            description='m added to the estimated window centre height. Positive '
                        'is higher. Use it if the detected quad sits off-centre '
                        'on the real aperture.'),
        DeclareLaunchArgument('approach_speed', default_value='0.30'),
        DeclareLaunchArgument(
            'traverse_speed', default_value='0.45',
            description='m/s through the window. The estimate is frozen by then.'),
        DeclareLaunchArgument(
            'align_tolerance', default_value='0.18',
            description='m radius around the approach point that counts as '
                        'lined up. Tighten it for a small window, but every '
                        'centimetre costs settling time against VO noise.'),
        DeclareLaunchArgument('align_yaw_tolerance_deg', default_value='8.0'),
        DeclareLaunchArgument(
            'align_settle_seconds', default_value='1.5',
            description='How long position, altitude and heading must ALL be in '
                        'tolerance together before the traverse commits.'),
        DeclareLaunchArgument('align_timeout', default_value='60.0'),
        DeclareLaunchArgument('traverse_timeout', default_value='25.0'),
        DeclareLaunchArgument(
            'clear_seconds', default_value='4.0',
            description='Station keeping on the far side before the descent.'),
        DeclareLaunchArgument(
            'blind_traverse_seconds', default_value='3.0',
            description='If vision dies mid-traverse, how long the aircraft may '
                        'push on open-loop along the committed heading before it '
                        'lands. This exists so it does not stop inside the '
                        'aperture. 0 disables it -- do not, unless you have a '
                        'reason.'),

        # ---- the estimator. The defaults are the ones the rejection logic was
        # tuned against; raise a threshold only after the log tells you which
        # test is doing the rejecting.
        DeclareLaunchArgument(
            'depth_min', default_value='0.35',
            description='m. Corner depths below this are not believed.'),
        DeclareLaunchArgument(
            'depth_max', default_value='8.0',
            description='m. Above this a gen-1 ZED is guessing.'),
        DeclareLaunchArgument(
            'corner_spread', default_value='0.25',
            description='m the four corner depths may differ from their median. '
                        'The primary outlier filter: a sample box that landed on '
                        'the wall behind the frame fails here.'),
        DeclareLaunchArgument(
            'corner_spread_frac', default_value='0.15',
            description='The same allowance as a fraction of range; the larger '
                        'of the two is used, so an obliquely-seen window at '
                        'distance is not rejected for being oblique.'),
        DeclareLaunchArgument(
            'plane_tolerance', default_value='0.15',
            description='m the reconstructed corners may sit off their own '
                        'best-fit plane. Backstop to corner_spread.'),
        DeclareLaunchArgument('window_min_size', default_value='0.35'),
        DeclareLaunchArgument(
            'window_max_size', default_value='3.0',
            description='m. With window_min_size, the sanity check on the '
                        'reconstructed aperture: a quad that comes out 6 m wide '
                        'is a bad depth, not a big window.'),
        DeclareLaunchArgument(
            'max_tilt_deg', default_value='35.0',
            description='How far off horizontal the window normal may be. A '
                        'window is vertical; a horizontal normal means the '
                        'detection is the floor or a ceiling light.'),
        DeclareLaunchArgument(
            'buffer_seconds', default_value='2.5',
            description='Length of the rolling window the median is taken over.'),
        DeclareLaunchArgument(
            'pose_min_samples', default_value='6',
            description='Accepted samples needed before the pose is flown to.'),
        DeclareLaunchArgument(
            'pose_max_age', default_value='1.5',
            description='s after which the newest accepted sample stops counting '
                        'as evidence about where the window is now.'),
        DeclareLaunchArgument(
            'pose_lost_timeout', default_value='6.0',
            description='s without a usable pose during the aim or the approach '
                        'before the attempt is abandoned into a landing.'),
        DeclareLaunchArgument(
            'gate_metres', default_value='1.0',
            description='m a new sample centre may sit from the current estimate '
                        'before it is rejected as an outlier.'),
        DeclareLaunchArgument('gate_yaw_deg', default_value='40.0'),

        # ---- the rest ----
        DeclareLaunchArgument(
            'reboot_fc', default_value='false',
            description='Reboot the FC first if EKF2 came up without the '
                        'rangefinder. Raise flight_node_delay to ~75 with this.'),
        DeclareLaunchArgument('flight_node_delay', default_value='12.0'),
        DeclareLaunchArgument('lcd', default_value='true'),
        DeclareLaunchArgument(
            'lcd_port', default_value='',
            description='Arduino serial port; empty = auto-detect.'),

        # flight:=false leaves the camera side running on its own, which is the
        # bench test. Grouped rather than conditioned individually because two
        # of these already carry a condition of their own.
        GroupAction([microxrce_node, lcd_node, reboot_node, bridge_node,
                     traverse_node],
                    condition=IfCondition(flight)),
        zed_wrapper,
        detect_node,
    ])
