"""
thermal_drop.launch.py, flown in the imav_indoor_2026 Gazebo world.

    ros2 launch drone_testing thermal_drop_sitl.launch.py

That is the whole thing: Gazebo + the scaled IMAV arena, the x500 spawned on
the SAME takeoff pad and the SAME heading as course_mission_sitl.launch.py,
PX4 SITL, MicroXRCEAgent on UDP, the ros_gz bridges for the two downward
sensors, and then thermal_drop.launch.py itself with fsm:=true, the
simulated thermal source, the downward ArUco detector, and every LENGTH
multiplied by 2.2.

THE MISSION, IN WORLD COORDINATES
---------------------------------
    takeoff_platform   (-4.40, -14.30)   ArUco id 0, armed on it
    platform_1         (-4.40,  +7.15)   ArUco id 2, the FIX
    box_red            (-0.44,  +6.38)   21 C
    box_blue           (+0.88,  +7.48)   70 C   <== the target
    box_purple         (+1.76,  +5.28)   21 C
    platform_2         (+4.40,  +7.15)   ArUco id 3, the DATUM
    landing_platform   (+4.35, -14.30)   ArUco id 1, the LANDING pad

    Nose along +y throughout, so "right" is +x.

      1. Arm on the pad, climb to 2.64 m (the real 1.20 m x 2.2). The
         whole outbound mission is flown at that height -- it is looking
         for markers on the floor, and low is better for that.
      2. MARK_SEARCH: forward along x = -4.40, at 2.64 m, for about 21.5 m
         until the downward camera finds platform_1. Everything in this arena that the
         aircraft could hit -- both walls, the bars, the tube gate -- stands
         between x = -1.65 and +1.65, so this lane is empty. The first 2.2 m
         do not count, because the pad it just left is also a marker. 24.2 m
         is the CAP: the real arena's pads are 8.7-8.8 m apart and fly a
         9.0 m cap, and this world's are 21.45 m apart. Reaching the cap
         means the marker is not there.
      3. MARK_HOVER: centre on id 2, hold 1 s (a TIME, so it is NOT scaled).
      4. BOX_OFFSET: 3.30 m RIGHT (the real 1.50 m; the real ARENA's own
         number is 2.20 m) to (-1.10, +7.15).
      5. SURVEY: climbs 2.64 -> 5.50 m over the boxes FIRST, and collects
         nothing on the way up. Then no search pattern any more: it holds
         still at 5.50 m, where
         the thermal footprint is about 15.7 m by 8.4 m and all three boxes
         are in one frame, watches for the dwell, and takes the hottest.
         Then APPROACH / DESCEND / HOVER down to 1.10 m (the real 0.50 m),
         LED blinking red the whole way, servo fired at the bottom.
      6. RETREAT: climb to 2.64 m, then 3.30 m RIGHT, which lands the
         aircraft at about (+4.18, +7.48) -- 0.39 m from platform_2.
      7. LAND_SEARCH / LAND_ALIGN: find id 3 and square up on it. It is a
         DATUM, not the pad: the arena's landing marker is at the OTHER end
         of the corridor.
      8. LAND_RETURN: backwards down the corridor, up to 24.2 m, nose still
         along +y -- no 180 deg turn, so the corridor heading the flight
         measured while stationary is still the one being flown -- until
         id 1 is under the camera at y = -14.30.
      9. LAND_CENTRE / LAND_DESCEND: centre on id 1, walk down to 0.99 m and
         hand the last of it to PX4.

WHY 2.2
-------
The simulated arena is the real arena scaled up by 2.2, and the x500 is NOT
scaled -- it is a real-sized aircraft in a world 2.2x too big. So:

  * everything that describes the WORLD is multiplied by 2.2: altitudes,
    the legs, the search radius, the cluster radius, the tolerances, the
    descent rates. Speeds are scaled too, so a leg takes the same wall-clock
    time it would on the real course.

  * everything that describes the AIRCRAFT is NOT: the camera offsets from
    the drop point, the servo, the LED.

  * TIMES are not lengths. mark_hover_seconds, hover_seconds,
    align_settle_seconds, survey_dwell_seconds, the stage timeouts and the
    flight clock are all left alone -- except where a leg got 2.2x longer in
    wall-clock terms and its timeout had to grow with it, which is called
    out where it happens below.

WHAT MAKES ONE BOX HOT
----------------------
Nothing in this file. imav2026_scaled.sdf.world gives box_blue's visuals
70 C and the other two 21 C through gz-sim-thermal-system; read the
"THE THREE BOXES" comment there before changing which one is the target.
The flight node is told nothing about it -- it surveys and decides.

WHAT THE AIRCRAFT CARRIES THAT THE COURSE AIRCRAFT DOES NOT
------------------------------------------------------------
x500_drone_thermal.urdf.xacro = x500_drone.urdf.xacro + thermal_cam.xacro +
down_cam.xacro. Two extra links on the belly, both looking down, both
bridged below. The course aircraft and the dark-room aircraft are untouched
and carry neither, and the dark room's 2D scanning lidar is in ITS overlay
and is not on this aircraft.

WHAT IT NEEDS
-------------
~/PX4-Autopilot built for px4_sitl_default, MicroXRCEAgent on PATH, and
imav_indoor_2026 built and sourced (this file reads its world, its models
and its x500_drone_thermal.urdf.xacro).

THE THERMAL CAMERA NEEDS ogre2, AND ONLY ogre2
-----------------------------------------------
gz-rendering implements thermal rendering in ogre2 and not in ogre1. The
world's Sensors system asks for ogre2 explicitly, which is what makes this
work even though GZ_SIM_RENDER_ENGINE is set to ogre for the GUI (same as
course_mission_sitl). If /thermal/raw is silent, check that line in
imav2026_scaled.sdf.world first -- the failure is silent, not an error.

USEFUL ARGUMENTS
----------------
    fsm:=false          fly thermal_drop instead: survey from the takeoff
                        pad, which over bare floor finds nothing. Useful
                        only for checking the sensor chain.
    mark_search:=false  skip the marker hunt and survey from the pad.
    precision_land:=false
                        plain PX4 land after the retreat.
    land_return:=false  land on the first marker after the retreat (id 3)
                        instead of flying the corridor home to id 1.
    aruco:=false        no marker detector at all: the MARK_* and LAND_*
                        stages creep their full distance, find nothing and
                        end honestly.
    headless:=true      no Gazebo GUI.
    release_enabled:=false
                        fly the whole thing but never move the servo.
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
    # The GUI's engine. The SENSORS' engine is ogre2, set in the world file,
    # and the thermal camera depends on that and not on this. See the header.
    set_render_engine = SetEnvironmentVariable('GZ_SIM_RENDER_ENGINE', 'ogre')

    # Every process this file starts -- the ROS nodes AND MicroXRCEAgent --
    # goes on a private DDS domain, for the reason course_mission_sitl's
    # ros_domain_id argument spells out at length: on a lab network any other
    # PX4 publishing /fmu/out/... into domain 0 is heard by every node here,
    # and it presents as an estimator gone mad rather than as a network fault.
    set_domain_id = SetEnvironmentVariable(
        'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id'))
    set_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(os.path.expanduser('~'), 'PX4-Autopilot', 'build',
                     'px4_sitl_default', 'src', 'modules', 'simulation',
                     'gz_plugins'))

    # ------------------------------------------------------------- the world
    # The THERMAL aircraft: the course airframe plus the two downward
    # sensors. course_mission_sitl.launch.py still processes the plain
    # x500_drone.urdf.xacro and is unaffected by anything here.
    robot_description = xacro.process_file(
        os.path.join(sim_share, 'description',
                     'x500_drone_thermal.urdf.xacro')).toxml()

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

    # On the takeoff pad, nose down the course (+y). THE SAME SPAWN AS
    # course_mission_sitl.launch.py, deliberately: the two missions start
    # from the same place on the same heading, and every leg of this one is
    # measured off that heading.
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
            # The forward OAK-D. Nothing in THIS mission reads it, but the
            # bridge is cheap and having it there means a window or bar
            # detector can be started by hand against the same simulation.
            '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

            # THE TWO DOWNWARD SENSORS THIS MISSION IS ABOUT.
            #
            #   down_cam.xacro and thermal_cam.xacro both set an explicit
            #   <topic>, so these are the gz topic names verbatim and not
            #   Gazebo's scoped /world/.../sensor/... form. That is safe for
            #   these two BECAUSE nothing inside PX4 subscribes to them -- it
            #   is exactly the rename that must never be done to the flow
            #   camera or the downward rangefinder below, whose scoped names
            #   are hard-coded in PX4's GZBridge.
            #
            #   /thermal/raw is mono16 (Gazebo L16), 32x24, counts of
            #   kelvin/0.01. thermal_sim turns it into the 32FC1 degrees-C
            #   /thermal/image the flight node reads.
            '/camera/down/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/thermal/raw@sensor_msgs/msg/Image[gz.msgs.Image',

            # The flow camera and the rangefinder keep Gazebo's DEFAULT
            # scoped topic names, because those are the ones PX4 subscribes
            # to. Bridged here purely so they can still be echoed.
            '/world/imav2026_scaled/model/x500_drone/link/flow_link'
            '/sensor/flow_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/imav2026_scaled/model/x500_drone/link/lidar_sensor_link'
            '/sensor/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
    )

    # PX4 SITL. The stock 4001 airframe plus config/px4_sitl_imav.rcS, the
    # same as the course sim. -w is required BECAUSE of -s: PX4 only picks
    # its own working directory when no startup file is given.
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
        'PX4_UXRCE_DDS_DOM_ID': LaunchConfiguration('ros_domain_id'),
        'PX4_BARO_CTRL': LaunchConfiguration('baro_fallback'),
    }
    # A PX4 killed rather than exited leaves /tmp/px4_lock-0 and
    # /tmp/px4-sock-0 behind, and the next run then prints only
    #     INFO [px4] PX4 server already running for instance 0
    # and exits -- no estimator, no VehicleLocalPosition, and a flight node
    # that sits there saying it cannot arm. pkill -x matches the process
    # NAME: -f would match the very shell running this string.
    px4_cmd = ('pkill -x px4 2>/dev/null; sleep 1; '
               f'rm -f /tmp/px4_lock-0 /tmp/px4-sock-0; '
               f'exec {px4_bin} -w {px4_rootfs} -s {px4_rcs}')

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

    # ---------------------------------------------------------- the mission
    mission = TimerAction(
        period=LaunchConfiguration('mission_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('drone_testing'), 'launch',
                'thermal_drop.launch.py')),
            launch_arguments={
                # THE WHOLE MISSION, not just the drop.
                'fsm': LaunchConfiguration('fsm'),
                'mode': 'fly',
                'agent_only': LaunchConfiguration('agent_only'),

                # ---- the sensors: Gazebo, not I2C and not /dev/video ----
                'thermal_sim': 'true',
                'thermal_raw_topic': '/thermal/raw',
                # Must equal <resolution> in thermal_cam.xacro.
                'thermal_resolution': '0.01',
                'thermal_noise_c': '0.1',
                'aruco': LaunchConfiguration('aruco'),
                'aruco_image_topic': '/camera/down/image_raw',
                # id 2 is platform_1, the fix on the way out. id 3 is
                # platform_2, which is the DATUM the run home is squared up
                # on, not the pad. id 1 is landing_platform at (4.35, -14.30),
                # 21.45 m back down the corridor from id 3 -- that is what
                # the aircraft actually lands on, so it has to be here or
                # LAND_RETURN would fly the whole cap and never see it.
                # id 0 is the takeoff pad and is deliberately NOT here --
                # the aircraft arms on it. It cannot be confused with id 1
                # either way: the two sit at the same y, 8.75 m apart across
                # the corridor, so only one of them is ever under the camera.
                'aruco_marker_ids': '1,2,3',
                # The marker plate in the world is 0.88 m square, which is the
                # real course's 0.40 m scaled. Its black border runs to the
                # plate edge, so this IS the side length solvePnP wants.
                'aruco_marker_size': s(0.40),
                # down_cam.xacro's 1.5708 rad.
                'aruco_hfov_deg': '90.0',
                'aruco_dict': 'DICT_5X5_50',
                # NOT the OpenCV default of 0.05. Every marker in this world
                # lies on a 1.10 m pad and is 0.88 m across, so the pad's own
                # square outline is a second candidate quad about 2% of the
                # frame away from the marker's -- and at 0.05 cv2.aruco
                # merges the two, keeps the PAD, samples the bits a cell off
                # and rejects it. Measured: with 0.05 not one frame over
                # platform_1 decoded; with 0.02 every frame decoded, shadow
                # of the airframe across the marker included.
                'aruco_min_marker_distance_rate': '0.02',
                'led': 'false',         # no WS2812B on a simulated GPIO; the
                                        # LED commands still go to /led/command
                                        # and can be echoed.

                # ---- the thermal camera's geometry ----
                'hfov_deg': '110.0',
                # NOT 75. Gazebo has square pixels, so a 32x24 image at 110
                # deg horizontal is 93.93 deg vertical whatever the real
                # MLX90640 does. thermal_cam.xacro derives it and explains
                # why; passing 75 here would put every box a fixed fraction
                # too far forward or back.
                'vfov_deg': '93.93',
                'cam_yaw_deg': '0.0',
                'flip_lr': 'false',
                'flip_ud': 'false',
                # AIRCRAFT geometry, so NOT scaled, and read straight off
                # thermal_cam.xacro: the camera is at (-0.02, 0) in base_link.
                # There is no release mechanism on the simulated airframe, so
                # the DROP POINT is defined to be directly under the camera --
                # which makes cam_from_drop zero and drop_from_cog the
                # camera's own offset. On the aircraft these are three
                # different numbers and must be measured, not copied.
                'cam_from_drop_forward': '0.0',
                'cam_from_drop_right': '0.0',
                'drop_from_cog_forward': '-0.02',
                'drop_from_cog_right': '0.0',
                'frame_latency': '0.06',
                'max_ray_angle_deg': '60.0',

                # ---- the arena ----
                # The hot surface is each box's INTERIOR FLOOR, 0.033 m above
                # the arena floor -- not the 0.29 m wall tops. The projection
                # intersects the ray with the plane at this height, so it is
                # the height of the thing the camera actually sees.
                'box_height': '0.033',
                'expected_boxes': '3',
                # The survey centre is (-1.10, +7.15) and the furthest box
                # (purple) is 3.40 m from it, so 4.4 takes all three and
                # still rejects anything from the next arena over.
                'search_radius': s(2.00),
                # The closest two boxes are 1.72 m apart in this world, so a
                # 0.66 m cluster radius cannot merge them.
                'cluster_radius': s(0.30),
                'min_cluster_frames': '3',
                'min_contrast': '3.0',
                'min_blob_pixels': '2',
                # box_blue leads by about 49 C, so this passes easily. Lower
                # the world's 343.15 if you want to make the choice hard.
                'min_hot_margin': '2.0',
                'verify_frames': '5',
                'verify_window': '4.0',
                'verify_ratio': '0.7',
                'retarget_margin': '1.5',

                # ---- the climb, the survey and the drop ----
                # 2.64. The real 1.20 m. Takeoff, the marker creep, the
                # hover and the sidestep are all flown here; only the
                # survey goes higher, and only once it is over the boxes.
                'cruise_altitude': s(1.20),         # 2.64
                # 5.50. The real mission now watches the boxes from 2.50 m
                # instead of flying a pattern at 1.50 m, and in a world 2.2x
                # too big the same view is 2.2x higher up. It is well inside
                # the arena: the tube gate and the bars are all below 3.5 m.
                'survey_altitude': s(2.50),         # 5.50
                # The ceiling moves with it, for the reason thermal_drop.py
                # gives: it is the clamp on survey_altitude, and a clamp
                # below the height the mission needs would silently undo the
                # change above.
                'max_survey_altitude': s(2.60),     # 5.72
                'drop_altitude': s(0.50),           # 1.10, the "50 cm" scaled
                'min_altitude': s(0.40),
                # Above survey_altitude, or the climb to the survey height
                # would be clamped short of it.
                'max_altitude': s(3.00),
                'survey_dwell_seconds': '4.0',      # a TIME
                # 0 = NO SEARCH PATTERN. From 5.50 m the thermal footprint is
                # about 15.7 m by 8.4 m and all three boxes -- spread over
                # 6.4 m of scaled arena -- are in one frame, so the aircraft
                # stops, watches for the dwell and ranks them. The five-point
                # ring is gone; see _begin_survey() in thermal_drop.py.
                'survey_step': '0.0',
                'approach_tolerance': s(0.15),
                'descend_tolerance': s(0.12),
                'descend_speed': s(0.12),
                'align_tolerance': s(0.08),
                'align_settle_seconds': '2.0',      # a TIME
                'hover_seconds': '2.0',             # a TIME
                'track_gate': s(0.40),

                # ---- the servo and the LED ----
                # PX4 SITL has no servo on the 4001 airframe, so the command
                # goes out, PX4 acks it and nothing moves. That is the point:
                # the state machine commits to the release at the right
                # moment and the ack is in the log to prove it.
                'release_enabled': 'true',
                'servo_command': 'set_actuator',
                'servo_index': '1',
                'servo_drop_value': '1.0',
                'servo_neutral_value': '-1.0',
                'servo_hold_seconds': '2.0',

                # ---- the legs thermal_fsm adds ----
                'mark_search': LaunchConfiguration('mark_search'),
                'mark_required': 'true',
                # The creep is 21.45 m of world (pad y = -14.30 to
                # platform_1 y = +7.15). 24.2 leaves a 2.75 m margin and
                # still stops short of the dark room's wall at y = +9.90.
                #
                # NOTE this is NOT the hardware number scaled. The real
                # arena's two markers are 8.7-8.8 m apart, so the aircraft
                # flies a 9.0 m cap; the same pair in this world is 21.45 m
                # apart, which is 9.75 m before scaling, not 8.75. The arena
                # was scaled from a slightly different layout. Scale the
                # SIM's own geometry, not the hardware's parameter.
                'mark_search_distance': s(11.00),
                'mark_search_speed': s(0.30),       # 0.66 m/s -> about 33 s
                'mark_min_travel': s(1.00),         # clear of the takeoff pad
                'mark_hover_seconds': '1.0',        # a TIME: the 1 s asked for
                'box_offset_right': s(1.50),        # 3.30
                # 33 s of creep plus the acceleration at each end. Not a
                # length, but it had to grow because the LEG did.
                'mark_stage_timeout': '90.0',
                # The carrot sits this far ahead ALONG the leg's line. It is
                # a world distance, so 1.54 m here, and it MUST stay above
                # move_leash (0.88) or the leash clamps it. At 1.54 m a 1 m
                # cross-track error commands a 33 deg correction; the old
                # far-end target commanded 2.7 deg and the aircraft finished
                # the marker run 1.09 m right of where it started.
                'leg_lookahead': s(0.70),
                # 0. The aim bias this was found chasing was EKF2's yaw
                # wandering, which is fixed at the source in
                # config/px4_sitl_imav.rcS (EKF2_MAG_TYPE 6), not trimmed
                # out here. Trim is for a bias that is real and repeatable.
                'leg_trim_deg': '0.0',
                # THE CORRIDOR IS DUE NORTH IN THE ESTIMATOR'S FRAME, and
                # this states it rather than inferring it.
                #
                #   The arena's +y is north, the aircraft is spawned exactly
                #   along it, and EKF2's x/y frame is world-aligned (measured:
                #   estimate and Gazebo truth agreed to 2 cm over 20 m). So
                #   the corridor's bearing is 0.
                #
                #   It is NOT what the heading estimate says. EKF2 datums yaw
                #   off the magnetometer's declination and x/y off flow, so
                #   they do not share a north: the heading read a rock-steady
                #   +5.95 deg with the nose physically on the corridor, and
                #   flying the leg on that number put the aircraft 2.40 m
                #   right over 23 m -- on a line the tracker held to 3 cm.
                #   Straight, but aimed wrong.
                #
                #   On the aircraft this stays nan: there is no survey of the
                #   hall, the operator's aim defines the corridor, and
                #   leg_trim_deg is the dial for the declination.
                'leg_bearing_deg': '0.0',

                # ---- the landing ----
                'precision_land': LaunchConfiguration('precision_land'),
                # The retreat already ends 0.39 m from platform_2, so this is
                # slack rather than a search.
                'land_search_distance': s(2.50),
                'land_search_speed': s(0.30),
                # THE RUN HOME. platform_2 (id 3) is the DATUM, not the pad:
                # the aircraft squares up on it and then flies the corridor
                # BACKWARDS to landing_platform (id 1) at (+4.35, -14.30),
                # which is what it lands on. That is 21.45 m -- the same
                # corridor length as the outbound creep, because it is the
                # same pair of pads mirrored across the arena -- so the cap
                # is the same s(11.00) = 24.20 m, and for the same reason it
                # is not the hardware 9.0 m scaled.
                #
                # Reaching the cap is NOT a failure: the cone is already in
                # the box, so the aircraft just lands where it is.
                'land_return': 'true',
                'land_return_distance': s(11.00),   # 24.20
                'land_return_speed': s(0.30),       # 0.66 -> about 33 s
                'land_return_min_travel': s(1.00),  # clear of the datum
                'pad_centre_tolerance': s(0.10),
                'pad_centre_seconds': '1.0',        # a TIME
                'pad_descent_rate': s(0.20),
                'pad_handoff_height': s(0.45),      # 0.99
                'pad_gain': '0.8',                  # a fraction, not a length
                'pad_max_nudge': s(0.30),
                'pad_lost_seconds': '2.0',          # a TIME
                'pad_stage_timeout': '45.0',
                'marker_anchor_gain': '0.35',       # a fraction, not a length
                'marker_anchor_max_age': '3.0',     # a TIME
                # A world height: the downwash comes off the pad at a scaled
                # distance like everything else here.
                'ground_effect_height': s(0.55),

                # ---- the retreat ----
                'retreat_altitude': s(1.20),        # 2.64
                # 3.30. The step RIGHT off the boxes and onto the corridor
                # platform_2 stands on. The real arena's number is 2.20 m
                # and this world's is 1.50 m before scaling -- the same
                # relationship BOX_OFFSET_RIGHT has, and for the same
                # reason: the two arenas put the pads at different distances
                # from the boxes. From box_blue at (+0.88, +7.48) this ends
                # at about (+4.18, +7.48), which is 0.39 m from platform_2,
                # so the datum marker is already in the down camera when
                # LAND_SEARCH starts. It was s(1.0) -- 2.20 m, which stopped
                # 1.10 m short of the pad and left LAND_SEARCH to find it by
                # creeping FORWARD, in the one direction the pad is not.
                'retreat_right': s(1.50),           # 3.30
                'retreat_timeout': '30.0',
                'land_after_drop': 'true',

                # ---- pace ----
                'ground_wait_seconds': '5.0',
                'hold_seconds': '4.0',
                'climb_speed': s(0.35),
                'land_speed': s(0.15),
                'move_speed': s(0.25),
                # The one that actually governs ground speed: the carrot is
                # capped this far ahead of the measured position and PX4
                # flies that capped error, so the speed is roughly
                # MPC_XY_P * move_leash whatever move_speed says. Left at the
                # airframe's 0.40 the course sim flew every leg at ~0.27 m/s
                # while being commanded 0.99. A world distance.
                'move_leash': s(0.40),
                'takeoff_accept_tolerance': s(0.30),
                'stage_timeout': '60.0',
                # The whole mission: about 8 s of climb to 5.50 m, 4 s of
                # hold, 33 s of creep, 1 s of hover, 5 s across, ~10 s of
                # survey (one observation now, not a five-point ring), the
                # drop, the retreat -- and then the run home, which is
                # another 33 s of corridor plus the align and the descent.
                # 600 s is not a budget, it is the point at which something
                # has clearly hung.
                'flight_seconds': '600.0',
                'flight_node_delay': '8.0',
            }.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true runs Gazebo with no GUI.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='78',
            description='DDS domain for the whole simulation, ROS nodes and '
                        'MicroXRCEAgent alike, and for PX4 via '
                        'UXRCE_DDS_DOM_ID. It is NOT cosmetic: ROS 2 and '
                        'uXRCE-DDS both default to domain 0 and discovery is '
                        'multicast over every interface, so any other PX4 on '
                        'the network publishes /fmu/out/... into the same '
                        'domain and every node here subscribes to BOTH '
                        'vehicles. Check with "ros2 topic info -v '
                        '/fmu/out/vehicle_local_position_v1": more than one '
                        'publisher means you are hearing someone else. NOTE: '
                        'a separate terminal needs ROS_DOMAIN_ID exported to '
                        'the same value to see this sim\'s topics. It is 78 '
                        'and not the course sim\'s 77 so that the two '
                        'simulations can be up at once without hearing each '
                        'other.'),
        DeclareLaunchArgument(
            'px4_terminal', default_value='false',
            description='true puts PX4 in its own gnome-terminal so you get '
                        'the interactive pxh> shell. The default runs it '
                        'inline, where its log is visible with everything '
                        'else but you cannot type at it.'),
        DeclareLaunchArgument(
            'agent_only', default_value='false',
            description='true brings up the sim, the thermal source and the '
                        'marker detector but NOT the flight node, so you can '
                        'run it by hand and keep the q/k keyboard aborts.'),
        DeclareLaunchArgument(
            'fsm', default_value='true',
            description='true flies thermal_fsm: marker, boxes, drop, marker, '
                        'land. false flies thermal_drop, which surveys from '
                        'the takeoff pad -- over bare floor, so it finds '
                        'nothing. Useful only for checking the sensor chain.'),
        DeclareLaunchArgument(
            'aruco', default_value='true',
            description='Start the downward ArUco detector. false makes the '
                        'MARK_* and LAND_* stages creep their full distance, '
                        'find nothing and end honestly, which is a legible '
                        'way to test the rest of the machine.'),
        DeclareLaunchArgument(
            'mark_search', default_value='true',
            description='false skips the marker hunt and surveys from the '
                        'takeoff pad instead.'),
        DeclareLaunchArgument(
            'precision_land', default_value='true',
            description='false lands with PX4 wherever the retreat ends, '
                        'instead of hunting for the landing marker.'),
        DeclareLaunchArgument(
            'baro_fallback', default_value='1',
            description='1 keeps EKF2_BARO_CTRL on, so the barometer carries '
                        'the height while the rangefinder is not being fused '
                        '(the range stays the height REFERENCE). 0 restores '
                        'the range-only configuration, where a dropout leaves '
                        'no height source at all.'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='12.0',
            description='Seconds to let Gazebo load the arena before the '
                        'aircraft is spawned into it.'),
        DeclareLaunchArgument(
            'px4_delay', default_value='22.0',
            description='Seconds before PX4 starts. It must be AFTER the '
                        'spawn: PX4_GZ_STANDALONE makes PX4 attach to an '
                        'existing model, and attaching to a world that is not '
                        'stepping yet corrupts its clock for the whole run, '
                        'which presents as "Preflight Fail: ekf2 missing '
                        'data" and an aircraft that disarms on the ground. If '
                        'Gazebo is on software rendering (libEGL warnings in '
                        'the log) it loads slower than these delays assume -- '
                        'raise all three.'),
        DeclareLaunchArgument(
            'mission_delay', default_value='40.0',
            description='Seconds before thermal_drop.launch.py is included. '
                        'Must leave PX4 time to boot and EKF2 time to '
                        'converge, or the flight node spends its first '
                        'seconds reporting an estimator that is still '
                        'starting up.'),

        # ON THE TAKEOFF PAD, nose along +y. The same spawn as
        # course_mission_sitl.launch.py. The arena floor runs to y = -15.4.
        DeclareLaunchArgument('spawn_x', default_value='-4.4'),
        DeclareLaunchArgument('spawn_y', default_value='-14.3'),
        DeclareLaunchArgument('spawn_z', default_value='0.5'),
        DeclareLaunchArgument('spawn_yaw', default_value='1.5708'),

        set_resource_path,
        set_render_engine,
        set_domain_id,
        set_plugin_path,
        robot_state_publisher,
        gazebo,
        gazebo_headless,

        # ORDER MATTERS, and not just for tidiness. PX4 must not start before
        # the model exists in a world that is already stepping. Started at
        # the same instant as Gazebo it latches onto whatever clock it finds
        # first and then logs
        #     ERROR [vehicle_imu] gyro timestamp error
        # after which the estimator never converges and the vehicle can never
        # arm.
        TimerAction(period=LaunchConfiguration('spawn_delay'),
                    actions=[spawn_entity, bridge]),
        TimerAction(period=LaunchConfiguration('px4_delay'),
                    actions=[px4, px4_in_terminal, dds_agent]),
        mission,
    ])
