"""
course_mission.launch.py, flown in the imav_indoor_2026 Gazebo world.

    ros2 launch drone_testing course_mission_sitl.launch.py

That is the whole thing: Gazebo + the scaled IMAV arena, the x500_drone
spawned facing the blue window of the exit wall, PX4 SITL, MicroXRCEAgent on
UDP, the ros_gz camera bridge, and then course_mission.launch.py itself with
agent:=false zed:=false and every LENGTH multiplied by 2.2.

WHY 2.2

    The simulated arena is the real arena scaled up by 2.2, and the x500 is
    NOT scaled -- it is a real-sized aircraft in a world 2.2x too big. So:

      * everything that describes the WORLD is multiplied by 2.2: altitudes,
        standoffs, the gaps between obstacles, the bar heights, the window
        size limits, the depth range, the position tolerances. Speeds are
        scaled too, so a stage takes the same wall-clock time it would on the
        real course.

      * everything that describes the AIRCRAFT is NOT: drone_width,
        drone_height, gear_below_camera and the camera mounting pose are the
        x500's own numbers, taken from x500_drone.urdf.xacro. The effect is
        that the simulated aircraft has 2.2x the clearance it really has --
        this sim proves the STATE MACHINE, not the clearances.

    Where a scaled hardware number and the world file disagreed, the world
    file wins: the bar heights, bar radii and tube geometry below are read
    straight out of imav2026_scaled.sdf.world, not multiplied up.

THE COURSE IN WORLD COORDINATES (flight is +y)

    exit wall, blue window   y = -12.1   centre x = -0.66, z = 2.86
    red bar                  y =  -9.9   z = 3.52, r = 0.044   (over)
    blue bar 1               y =  -7.7   z = 1.76, r = 0.0396  (under)
    blue bar 2               y =  -5.5   z = 1.76               (under)
    tube gate (tube_B)       y =  -2.2   uprights at x = +-1.1
    lone upright (tube_A)    y =   0.0   x = 0

    The tube gate's diagonal runs from (x = -1.10, z = 3.75) down to
    (x = +1.10, z = 1.67) -- work it out from tube_B_diag in the world file:
    pose z 2.7142, pitch -0.8229 rad, length 3.058. The cross bar is at
    z = 1.01. Flying +y the aircraft's left is -x, so the HIGH end of the
    diagonal, and with it the big opening, is on the aircraft's LEFT, and the
    shift afterwards is to the LEFT as well -- away from tube_A at x = 0,
    which is then off to the right.

    This was the wrong way round here until 2026-09-20 (it said low at -x,
    and set gap_side 'right' with a negative shift), which aimed the aircraft
    at the short side and flew it into the diagonal. gap_side is 'auto' now
    and works it out from the two diagonal heights; they still have to be
    given the right way round, and the camera overrules both anyway when it
    can see the opening.

THE TUBE GATE, AND WHY IT IS OFF BY DEFAULT

    course_fsm's tube stage models the obstacle as THREE coplanar uprights at
    equal spacing, and flies the gap between the middle one and one side.
    solve_gap() fits that layout and refuses anything else: with only two
    uprights both "which one is the middle?" answers fit equally well and it
    returns "two layouts fit equally well", and a fit that matches fewer than
    tube_min_matched uprights is rejected outright.

    This world does not contain that obstacle. tube_B at y = -2.2 is a TWO
    post gate (x = +-1.1) with a cross bar and a diagonal, and tube_A at
    y = 0.0 is a single post in a different plane 2.2 m further on. So the
    solve cannot succeed here however the parameters are set, and turning
    tubes on only buys a 90 s TUBE_SEARCH before the aircraft gives up and
    lands. That is exactly what it did: tube_detect saw both uprights quite
    happily (d=3.67 az=-5, d=3.74 az=+9) and solve_gap still had nothing to
    fit them to.

    The scaled tube parameters below are therefore correct-as-derived and
    left in place, ready for a world whose tube obstacle matches the course.
    Nothing in course_fsm needs changing.

WHAT IT NEEDS

    ~/PX4-Autopilot built for px4_sitl_default, MicroXRCEAgent on PATH, and
    imav_indoor_2026 built and sourced (this file reads its world, its models
    and its x500_drone.urdf.xacro).

USEFUL ARGUMENTS

    tubes:=true         attempt the tube gate as well (it will not solve --
                        read THE TUBE GATE below before turning it on)
    window_after_tubes:=true
                        after the tubes, fly the WHOLE window mission a second
                        time -- search, lock, align, traverse -- and only then
                        the landing the course normally ends with. It only
                        runs once the tubes FINISH, so it needs tubes:=true
                        and a tube obstacle that solves; and it is flown on
                        what the camera measures, so it needs a window
                        standing past the gate. In THIS world neither holds:
                        the gate does not solve (see THE TUBE GATE) and there
                        is no second window past it.
    pad_right:=3.3      how far RIGHT the aircraft steps off the obstacle line
                        before creeping forward for the marker. A world
                        distance: the default is the real course's 1.5 m
                        scaled by 2.2.
    agent_only:=true    bring the sim and the detectors up but NOT the flight
                        node, so you can run course_fsm by hand with the q/k
                        keyboard aborts
    headless:=true      no Gazebo GUI
    spawn_y:=-15.0      how far back of the wall the aircraft starts
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro

# The arena is this many times bigger than the real one.
SCALE = 2.2


def s(value):
    """A real-course length (or speed) as the simulated world sees it."""
    return str(round(float(value) * SCALE, 4))


def generate_launch_description():
    sim_pkg = 'imav_indoor_2026'
    sim_share = get_package_share_directory(sim_pkg)
    world_dir = os.path.join(os.path.expanduser('~'), 'ros2_humble_ws', 'src',
                             sim_pkg, 'world')

    # ------------------------------------------------------------ Gazebo env
    resource_path = ':'.join([
        os.path.dirname(sim_share),
        world_dir,
        os.path.join(world_dir, 'models'),
    ])
    set_resource_path = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                                  resource_path)
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre')

    # Every process this file starts -- the ROS nodes AND MicroXRCEAgent --
    # goes on a private DDS domain. See "ros_domain_id" below for why.
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id'))
    set_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(os.path.expanduser('~'), 'PX4-Autopilot', 'build',
                     'px4_sitl_default', 'src', 'modules', 'simulation',
                     'gz_plugins'))

    # ------------------------------------------------------------ the world
    robot_description = xacro.process_file(
        os.path.join(sim_share, 'description', 'x500_drone.urdf.xacro')).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    world_file = os.path.join(sim_share, 'world', 'imav2026_scaled.sdf.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch',
            'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('headless')),
    )

    # Square on the blue window's centreline, behind it, nose down the course.
    # The whole mission is flown off this heading, exactly as the real one is
    # flown off wherever the aircraft was pointed when it armed.
    spawn_entity = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', 'x500_drone',
                   '-topic', 'robot_description',
                   '-x', LaunchConfiguration('spawn_x'),
                   '-y', LaunchConfiguration('spawn_y'),
                   '-z', LaunchConfiguration('spawn_z'),
                   '-Y', LaunchConfiguration('spawn_yaw')],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # The flow camera and the lidar both keep Gazebo's DEFAULT scoped
            # topic names, because those are the ones PX4 subscribes to --
            # the flow plugin for the camera it derives flow from, and
            # GZBridge for the rangefinder. Renaming either one in the
            # xacro silently disconnects it from PX4. Bridged here under
            # their full names purely so they can still be echoed.
            '/world/imav2026_scaled/model/x500_drone/link/flow_link'
            '/sensor/flow_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/imav2026_scaled/model/x500_drone/link/lidar_sensor_link'
            '/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # PX4 SITL. The stock 4001 airframe, plus config/px4_sitl_imav.rcS, which
    # sources the normal rcS and then sets the handful of parameters this
    # course needs -- see that file. -w is required BECAUSE of -s: PX4 only
    # picks its own working directory when no startup file is given.
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_rcs = os.path.join(get_package_share_directory('drone_testing'),
                           'config', 'px4_sitl_imav.rcS')
    px4_env = {
        'PX4_GZ_WORLD': 'imav2026_scaled',
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': '4001',
        'PX4_GZ_MODEL_NAME': 'x500_drone',
        # Read by config/px4_sitl_imav.rcS, which puts it in UXRCE_DDS_DOM_ID
        # so PX4's DDS participants land on the same private domain as the
        # ROS side.
        'PX4_UXRCE_DDS_DOM_ID': LaunchConfiguration('ros_domain_id'),
    }
    # A PX4 killed rather than exited leaves /tmp/px4_lock-0 and
    # /tmp/px4-sock-0 behind, and the next run then prints only
    #     INFO [px4] PX4 server already running for instance 0
    # and exits -- no estimator, no VehicleLocalPosition, and a flight node
    # that sits there saying it cannot arm. Clearing both is part of starting
    # a simulation, so it happens here rather than being something you have
    # to remember.
    # pkill -x matches the process NAME, not the command line: -f would match
    # the very shell running this string and kill it.
    px4_cmd = ('pkill -x px4 2>/dev/null; sleep 1; '
               f'rm -f /tmp/px4_lock-0 /tmp/px4-sock-0; '
               f'exec {px4_bin} -w {px4_rootfs} -s {px4_rcs}')

    # Inline by default, so PX4's own boot log and any parameter complaint
    # lands in the same place as everything else. The first SITL run hid it
    # in a gnome-terminal, which is why the failsafe and the missing
    # rangefinder had to be inferred from the ROS side.
    px4 = ExecuteProcess(
        cmd=['bash', '-c', px4_cmd],
        cwd=px4_dir,
        additional_env=px4_env,
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('px4_terminal')),
    )
    px4_in_terminal = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c',
             ''.join(f'export {k}={v}; ' for k, v in px4_env.items())
             + f'cd {px4_dir} && {px4_cmd}; exec bash'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('px4_terminal')),
    )

    dds_agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                               output='screen')

    # --------------------------------------------------------- the mission
    # Held off until PX4 has booted and the bridge is publishing; the flight
    # node inside has its own flight_node_delay on top of this.
    mission = TimerAction(
        period=LaunchConfiguration('mission_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('drone_testing'), 'launch',
                'course_mission.launch.py')),
            launch_arguments={
                # The sim provides the DDS link and the camera itself.
                'agent': 'false',
                'zed': 'false',
                'reboot_fc': 'false',
                'lcd': 'false',
                'agent_only': LaunchConfiguration('agent_only'),

                # ---- camera: the OAK-D Lite on x500_drone, via ros_gz ----
                'image_topic': '/camera/rgb/image_raw',
                'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/rgb/camera_info',
                'depth_scale': '1.0',
                'fallback_hfov_deg': '73.0',        # 1.274 rad, if CameraInfo is late
                'max_fps': '15.0',
                'publish_image': 'true',
                'publish_compressed': 'false',

                # Camera pose in the body frame, from oakd_lite.xacro:
                # camera_mount_joint (.12 .03 0) + the sensor pose
                # (0.01233 -0.03 0.01878). AIRCRAFT geometry -- not scaled.
                'cam_x': '0.132',
                'cam_y': '0.0',
                'cam_z': '0.019',
                'cam_roll': '0.0',
                'cam_pitch': '0.0',
                'cam_yaw': '0.0',

                # ---- the aircraft: x500, real size, NOT scaled ----
                'drone_width': '0.50',
                'drone_height': '0.25',
                'gear_below_camera': '0.12',

                # ---- the climb and the sweep ----
                'takeoff_altitude': s(1.2),         # 2.64, window centre is 2.86
                'min_altitude': s(0.4),
                'max_altitude': s(3.0),
                'climb_speed': s(0.35),
                'land_speed': s(0.15),
                'ground_wait_seconds': '5.0',
                'flight_seconds': '600.0',

                # ---- the traversal ----
                # standoff is 1.2 m of real course rather than 1.6: the arena
                # floor ends at y = -15.4 and there is nowhere to back up to.
                'standoff_distance': s(1.2),
                'exit_distance': s(0.5),
                'vertical_clearance': s(0.15),
                'lateral_clearance': s(0.15),
                'hard_clearance': s(0.03),
                'sill_bias': s(0.10),
                'align_alt_tolerance': s(0.08),
                'align_tolerance': s(0.18),
                'align_cross_tolerance': s(0.06),
                'align_along_tolerance': s(0.25),
                'approach_speed': s(0.30),
                'traverse_speed': s(0.45),
                'recentre_backoff': s(0.60),
                'blind_traverse_seconds': '3.0',

                # The one that actually governs how fast the aircraft moves.
                # The carrot is capped at this distance ahead of the measured
                # position, and PX4 flies that capped error, so the speed is
                # roughly MPC_XY_P * move_leash no matter what the speed
                # arguments above say. Left at the airframe's 0.40 the
                # aircraft flew every leg at ~0.27 m/s while being commanded
                # 0.99, and the blue crossing timed out 1.18 m short of its
                # target. It is a world distance, so it scales with the world.
                'move_leash': s(0.40),

                # Stage timeouts, scaled for the same reason: each leg is 2.2x
                # longer. They are not tight limits, they are the point at
                # which a stage is declared stuck, so they are scaled and then
                # left generous.
                'traverse_timeout': '55.0',
                'align_timeout': '90.0',
                'recentre_timeout': '45.0',
                'course_vertical_timeout': '55.0',
                'course_cross_timeout': '45.0',
                'tube_search_timeout': '90.0',

                # ---- the estimator ----
                'depth_min': s(0.35),
                'depth_max': s(8.0),                # 17.6, depth far clip is 19.1
                'corner_spread': s(0.25),
                'plane_tolerance': s(0.15),
                'window_min_size': s(0.35),
                'window_max_size': s(3.0),
                'gate_metres': s(1.0),
                'side_mismatch': s(0.40),

                # ---- the course, straight out of imav2026_scaled.sdf.world ----
                'window_to_red_distance': '2.2',    # -12.1 -> -9.9
                'red_to_blue_distance': '2.2',      #  -9.9 -> -7.7
                'blue_bar_gap': '2.2',              #  -7.7 -> -5.5
                'blue_exit_distance': s(0.8),
                'red_bar_height': '3.52',
                'blue_bar_height': '1.76',
                'bar_radius': '0.044',
                'red_clearance': s(0.30),
                'blue_clearance': s(0.20),
                'course_climb_speed': s(0.40),
                'course_descent_speed': s(0.30),
                'bar_cross_speed': s(0.35),
                'course_xy_tolerance': s(0.12),
                'course_alt_tolerance': s(0.08),

                # ---- the tube gate, likewise from the world file ----
                'tubes': LaunchConfiguration('tubes'),
                # The blue crossing already ends 1.54 m short of the gate
                # (blue bar 2 at y = -5.5 + blue_exit 1.76), so the look-from
                # point is BEHIND where it stops: 3.30 - 1.76 - 2.64 = -1.10 m.
                # blue_to_tube_distance used to be 3.3 here and was applied
                # from the END of the blue crossing, which put the look-from
                # point 1.76 m past the gate and flew the aircraft into it.
                'blue_to_tube_distance': '0.0',     # 0 = use the two below
                'tube_plane_from_blue': '3.3',      # blue bar 2 -5.5 -> gate -2.2
                'tube_look_standoff': s(1.20),
                'tube_spacing': '2.2',              # uprights at x = +-1.1
                'tube_radius': '0.0495',
                'cross_bar_height': '1.0142',
                # See the header: -x is the aircraft's LEFT and is the HIGH
                # end, straight off tube_B_diag's pose in the world file.
                'diagonal_left_height': '3.754',
                'diagonal_right_height': '1.674',
                'gap_side': 'auto',                 # -> left, the tall side
                'tube_shift_left': '0.88',          # left, away from tube_A at x = 0
                'tube_standoff': s(1.20),
                'tube_pass_exit': s(0.50),
                'tube_exit_distance': s(0.0),   # = past tube_A, not at it
                'tube_back_distance': '2.2',    # tube_A at y = 0, gate at -2.2
                'tube_back_clear': s(1.00),
                'tube_clearance': s(0.12),
                'tube_cross_drop': s(0.15),
                'tube_cross_left': s(0.05),
                # BACK TO THE NUMBERS THAT TRAVERSED THE TUBES.
                # 0.90 -> 1.98 m. It leaves only 16 mm between the belly and
                # the top of the blue bar, but the aircraft stops 0.66 m in
                # FRONT of that bar and never passes over it, and raising it
                # is what broke the stage twice: at 2.42 m the camera's
                # horizon drops far enough down the frame that the uprights'
                # lower ends are lost against the red floor and every one of
                # them is rejected as "does not reach the floor".
                'tube_look_altitude': s(0.90),
                # 0 = no cap on backing up, so the full 1.10 m back-off runs
                # and the scan happens from 2.64 m -- the stand that works.
                # Capped to 0.44 m it sits 1.98 m out, where a post leaves the
                # frame during the sweep and no cell is ever enclosed.
                'tube_blue_clear': '0.0',
                'tube_exit_forward': s(1.50),
                'tube_scan_seconds': '8.0',
                'tube_scan_max_seconds': '20.0',
                'tube_gap_prefer': 'left',
                # ---- the pad landing ----
                # ON, so the stages after the tubes are actually flown, but
                # this world has nothing to land ON: x500_drone carries only
                # the forward OAK-D, and the aruco markers in imav2026_scaled
                # belong to ring_board_assembly, standing upright at z = 1.1,
                # not lying on the floor. So expect: 1 m right, a creep
                # forward, no marker, and a landing at the end of the creep.
                # To make it real, add a downward sensor to
                # x500_drone.urdf.xacro, bridge it, put a marker on the floor
                # past the tubes, and set pad_image_topic to the bridged topic.
                # An ARGUMENT rather than a constant, so pad:=false on the
                # command line reaches the flight node instead of being
                # silently dropped here -- which is what happened on the
                # 2026-09-20 run: it flew the pad stages anyway.
                'pad': LaunchConfiguration('pad'),
                # ... but no detector: there is no /dev/video here, and
                # aruco_pose opening a camera that does not exist is noise at
                # best. The pad stages fly, find nothing, and land.
                'pad_detector': 'false',
                'window_after_tubes': LaunchConfiguration('window_after_tubes'),
                'window2_exit_distance': LaunchConfiguration('window2_exit_distance'),
                'window2_altitude': LaunchConfiguration('window2_altitude'),
                'window2_hold_seconds': LaunchConfiguration('window2_hold_seconds'),
                'scan_span_deg': LaunchConfiguration('scan_span_deg'),
                'pad_right': LaunchConfiguration('pad_right'),
                'pad_search_distance': s(2.50),
                'pad_search_speed': s(0.30),
                'tube_cross_tolerance': s(0.05),
                'tube_depth_min': s(0.40),
                'tube_depth_max': s(6.00),
                'min_tube_top_height': s(1.20),
                # Scaled like the rest, but note it would need relaxing to
                # about 3.0 to work here even with a third upright: the arena
                # floor is red (0.80 0.06 0.06) and so are the uprights, so
                # the part of an upright below the camera's horizon is
                # silhouetted against a background of its own colour and never
                # reaches the mask. Every candidate is then rejected as "does
                # not reach the floor", which is what the first tube attempt
                # logged 632 times. At the 1.98 m scan altitude enough of each
                # upright stays above the horizon for this to pass, which is
                # why the traversal worked with it; it only became the
                # blocking rejection when the scan altitude went up. Back to
                # the tight value, with the altitude back where it belongs.
                'max_tube_bottom_height': s(0.40),
                'tube_cluster_radius': s(0.15),
            }.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true runs Gazebo with no GUI.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='77',
            description='DDS domain for the whole simulation, ROS nodes and '
                        'MicroXRCEAgent alike, and for PX4 via '
                        'UXRCE_DDS_DOM_ID. It is NOT cosmetic. ROS 2 and '
                        'uXRCE-DDS both default to domain 0 and discovery is '
                        'multicast over every interface, so on a lab network '
                        'any other PX4 -- a teammate\'s SITL, or this team\'s '
                        'own aircraft powered up on the bench -- publishes '
                        '/fmu/out/... into the same domain and every node '
                        'here subscribes to BOTH vehicles. That presents as '
                        'an estimator gone mad rather than as a network '
                        'fault: thousands of alternating "EKF2 HEADING reset" '
                        'lines, local_position_invalid flapping, and "Arming '
                        'denied: Resolve system health failures first", while '
                        "PX4's own uORB is steady and healthy throughout. "
                        'Check for it with "ros2 topic info -v '
                        '/fmu/out/vehicle_local_position_v1": more than one '
                        'publisher means you are hearing someone else. NOTE: '
                        'a separate terminal needs ROS_DOMAIN_ID exported to '
                        'the same value to see this sim\'s topics.'),
        DeclareLaunchArgument(
            'px4_terminal', default_value='false',
            description='true puts PX4 in its own gnome-terminal so you get '
                        'the interactive pxh> shell. The default runs it '
                        'inline, where its log is visible with everything '
                        "else but you cannot type at it."),
        DeclareLaunchArgument(
            'agent_only', default_value='false',
            description='true brings up the sim and the detectors but NOT '
                        'course_fsm, so you can run it by hand and keep the '
                        'q/k keyboard aborts.'),
        DeclareLaunchArgument(
            'tubes', default_value='false',
            description='true attempts the tube gate after the blue bars. It '
                        'will not solve in THIS world -- see THE TUBE GATE in '
                        'the header -- so the default lands after the bars, '
                        'which is the whole course this arena contains.'),
        DeclareLaunchArgument(
            'pad', default_value='true',
            description='false = land where the last obstacle finishes, with '
                        'no step right and no creep forward. This world has '
                        'nothing to land ON, so the pad stages here only fly '
                        'the pattern and land at the end of it.'),
        DeclareLaunchArgument(
            'window2_altitude', default_value='0.0',
            description='m the aircraft climbs back to after the tubes before '
                        'looking for the second window. A WORLD altitude. '
                        f'0 = takeoff_altitude, which here is {s(1.2)} m (the '
                        f'real course\'s 1.2 m scaled by {SCALE}) -- the '
                        'height the first window was searched for from.'),
        DeclareLaunchArgument(
            'window2_hold_seconds', default_value='2.0',
            description='s stationary at that altitude before the search '
                        'starts. Not a length: it is not scaled.'),
        DeclareLaunchArgument(
            'window2_exit_distance', default_value=s(1.00),
            description='m beyond the SECOND window its traverse ends. A '
                        'world distance. 0 keeps exit_distance, which the '
                        'course pins to the window-to-red midpoint.'),
        DeclareLaunchArgument(
            'scan_span_deg', default_value='20.0',
            description='Total yaw arc swept when searching for a window, '
                        'about the heading the aircraft is holding. It covers '
                        'the SECOND window search as well, which starts from '
                        'wherever the tubes left the aircraft pointing.'),
        DeclareLaunchArgument(
            'window_after_tubes', default_value='false',
            description='true flies a SECOND window traversal -- the blue '
                        'window -- after the tubes, and only then the landing '
                        'the course finishes with. It is the whole window '
                        'mission again (search, lock, align, traverse), flown '
                        'on what the camera measures, so it needs a window '
                        'actually standing past the tubes; with tubes:=false '
                        'the tubes never finish and this never runs.'),
        DeclareLaunchArgument(
            'pad_right', default_value=s(1.50),
            description='m to the RIGHT after the last obstacle before the '
                        'creep forward looking for the marker. A WORLD '
                        'distance, so the default is the real course\'s 1.5 m '
                        f'scaled by {SCALE}.'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='12.0',
            description='Seconds to let Gazebo load the arena before the '
                        'aircraft is spawned into it.'),
        DeclareLaunchArgument(
            'px4_delay', default_value='22.0',
            description='Seconds before PX4 starts. It must be AFTER the '
                        'spawn: PX4_GZ_STANDALONE makes PX4 attach to an '
                        'existing model, and attaching to a world that is not '
                        'stepping yet corrupts its clock for the whole run. '
                        'Raised from 14 s after two runs where PX4 booted '
                        'with "Preflight Fail: ekf2 missing data" and EKF2 '
                        'then dropped local position on the ground, which '
                        'disarms the aircraft before it ever takes off. If '
                        'Gazebo is on software rendering (libEGL warnings in '
                        'the log) it loads slower than these delays assume -- '
                        'raise all three again.'),
        DeclareLaunchArgument(
            'mission_delay', default_value='40.0',
            description='Seconds before course_mission.launch.py is included. '
                        'Must leave PX4 time to boot and EKF2 time to '
                        'converge, or the flight node spends its first '
                        'seconds reporting an estimator that is still '
                        'starting up.'),

        # Behind the blue window of the exit wall (x = -0.66, y = -12.1),
        # nose along +y. The arena floor runs to y = -15.4.
        DeclareLaunchArgument('spawn_x', default_value='-0.66'),
        DeclareLaunchArgument('spawn_y', default_value='-15.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.3'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708'),

        set_resource_path,
        set_render_engine,
        set_domain_id,
        set_plugin_path,
        robot_state_publisher,
        gazebo,
        gazebo_headless,

        # ORDER MATTERS, and not just for tidiness. PX4 must not start before
        # the model exists in a world that is already stepping. Started at the
        # same instant as Gazebo it latches onto whatever clock it finds first
        # and then logs
        #     ERROR [vehicle_imu] gyro timestamp error
        #     timestamp_sample: 819000, previous timestamp_sample: 294172000
        # after which the estimator never converges: z_valid stays false,
        # vz reads tens of thousands of m/s, and the vehicle can never arm.
        TimerAction(period=LaunchConfiguration('spawn_delay'),
                    actions=[spawn_entity, bridge]),
        TimerAction(period=LaunchConfiguration('px4_delay'),
                    actions=[px4, px4_in_terminal, dds_agent]),
        mission,
    ])
