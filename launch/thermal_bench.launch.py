#!/usr/bin/env python3
"""BENCH TEST OF THE DROP: hot box -> LED -> servo. Nothing flies.

    agent + thermal_sensor + thermal_bench + servo_controller + led_status

Four small nodes and no flight node at all.  There is no offboard stream, no
arm request and no setpoint published anywhere in this launch file, so it is
safe to run with the aircraft in your hands and the props off.  You do the
descending; the software does the seeing, the glowing and the dropping.

HOW TO RUN IT

  1. Find the servo's PX4 output FUNCTION number first, because on the bench
     that is the only way the servo will move at all (see WHY, below):

         ros2 run drone_testing servo_test --ros-args -p command:=sweep

     It steps MAV_CMD_ACTUATOR_TEST across Servo 1-8 = functions 201-208, one
     per second, printing PX4's ack.  Note the one that moves your servo.

  2. Props off.  Cone loaded.  Hot box on the floor.  Then:

         ros2 launch drone_testing thermal_bench.launch.py servo_function:=204

  3. Open  http://<this host>:8082/  on a laptop on the same network.  That is
     thermal_sensor's live view: the 32x24 frame in false colour, a THICK
     GREEN BOX round the blob the software has chosen, a yellow cross on the
     centroid it measures from, thin grey boxes round the warm things it is
     REJECTING, and a white crosshair at frame centre -- which with the lens
     level is straight down.

  4. Hold the aircraft above the hot box and lower it slowly.  Watch:

         nothing hot ............... LED off
         hot box, off to a side .... LED BLINK BLUE   -- move it over the box
         box under the camera ...... LED BLINK RED    -- the mission's own
                                                         DESCEND signal
         at drop_altitude .......... LED SOLID GREEN, SERVO OPENS

     The servo goes back to neutral and the LED off servo_hold_seconds later.

WHY THE SERVO NEEDS servo_function HERE

    PX4 ignores MAV_CMD_DO_SET_ACTUATOR while DISARMED, and nothing in this
    launch file arms anything.  So the bench uses MAV_CMD_ACTUATOR_TEST
    instead -- the command QGC's Actuators tab uses, which does work disarmed
    -- and that one addresses the output by its FUNCTION number rather than by
    the offboard actuator set.  servo_command is therefore defaulted to
    actuator_test here, the opposite of the flight default.  Leave
    servo_function at 0 and PX4 will refuse the command and say so in the log.

WHAT THIS DOES AND DOES NOT TEST

    Tests:      the MLX90640, find_blobs and its thresholds, WHICH box gets
                picked, the pixel->metres geometry and its flips, the
                rangefinder height, the LED wiring and the whole release path
                (thermal_bench -> /servo/drop -> servo_controller -> PX4).
    Does not:   attitude compensation (nothing is tilting), the survey ring,
                the ArUco legs, or the flight node's state machine. The bench
                assumes you are holding it roughly level.

IF IT WILL NOT DROP

    require_altitude:=false   fires on centring alone -- use this when there
                              is no rangefinder, or to check the servo and
                              LEDs without getting the height right.
    rearm_seconds:=5.0        loop instead of stopping after one drop, so you
                              can reload and go again without a restart.
    release_enabled:=false    everything but the servo: LEDs and logs only.
    ros2 topic echo /thermal/bench      the node's running commentary.

    Wrong box picked, or the offset the wrong way round?  That is a real
    finding -- the mission would do the same.  Fix min_contrast /
    min_blob_pixels / flip_lr / flip_ud / cam_yaw_deg here, then carry the
    same values into thermal_drop.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    L = LaunchConfiguration

    args = [
        ('agent', 'true',
         'Run the serial uXRCE-DDS agent. false if one is already up, or if '
         'you only want to see the thermal detection and the LEDs and do not '
         'care about the servo.'),

        # ---- detection. THE SAME NUMBERS AS THE FLIGHT NODE, always ----
        ('refresh_hz', '8', 'MLX90640 frame rate.'),
        ('stream_port', '8082', 'The browser view. 0 turns it off.'),
        ('jpeg_quality', '70', ''),
        ('publish_preview', 'false',
         'Also publish the annotated frame on thermal/preview, for rviz. The '
         'MJPEG stream is the cheap way to watch it.'),
        ('min_contrast', '3.0',
         'deg C above ambient before a pixel counts as warm.'),
        ('min_blob_pixels', '2',
         'Blobs smaller than this are noise, not boxes.'),
        ('hfov_deg', '110.0', 'MLX90640 field of view, across.'),
        ('vfov_deg', '75.0', 'MLX90640 field of view, down.'),
        ('cam_yaw_deg', '0.0', 'Thermal camera yaw vs the airframe nose.'),
        ('flip_lr', 'false', 'Set if RIGHT and LEFT come out swapped.'),
        ('flip_ud', 'false', 'Set if FORWARD and BACK come out swapped.'),

        # ---- when to drop ----
        ('drop_altitude', '0.50',
         'm above the BOX TOP at which the servo fires. The rangefinder reads '
         'height above whatever is under it, which over a box is the box top, '
         'which is the number that matters.'),
        ('altitude_tolerance', '0.15', 'm either side of drop_altitude.'),
        ('centre_tolerance', '0.20',
         'm. How close to straight down the box must be. Wider than the '
         'flight node\'s approach_tolerance on purpose: a hand is not a '
         'position controller.'),
        ('settle_seconds', '1.0',
         's it must stay centred and at height before firing, so a wobble on '
         'the way past does not drop the cone.'),
        ('require_altitude', 'true',
         'false = drop on centring alone, ignoring height. For a bench with '
         'no rangefinder, or to test the servo and LEDs on their own.'),
        ('lost_seconds', '1.0',
         's without a hot blob before it gives up and goes back to SEARCH.'),
        ('rearm_seconds', '0.0',
         's cooldown then go again. 0 = stop after one drop.'),

        # ---- the release ----
        ('release_enabled', 'true',
         'false = run everything but never move the servo. LEDs and logs only.'),
        ('drop_trigger_topic', '/servo/drop', 'std_msgs/Bool.'),
        ('servo_index', '1', 'PX4 "Offboard Actuator Set N", for set_actuator.'),
        ('servo_function', '0',
         'What goes in MAV_CMD_ACTUATOR_TEST param5. REQUIRED on the bench. '
         'NOT the "Servo 4" you read off QGC\'s Actuators tab: param5 is a '
         'MAVLink ACTUATOR_OUTPUT_FUNCTION, so Servo 1-8 is 33-40, or '
         '1201-1208 for PX4\'s own numbering. Do not work it out -- run '
         '`ros2 run drone_testing servo_test --ros-args -p command:=sweep` '
         'and use the number it prints for the step that moved the servo.'),
        ('servo_command', 'actuator_test',
         'actuator_test = what QGC uses, and the ONLY one that works while '
         'disarmed, so it is the bench default. set_actuator = what the '
         'mission uses in the air.'),
        ('servo_drop_value', '1.0', 'Actuator value that opens it, -1..1.'),
        ('servo_neutral_value', '-1.0', 'Actuator value it returns to.'),
        ('servo_hold_seconds', '2.0', 's the servo is held open.'),
        ('servo_close_on_start', 'false',
         'Send neutral once at startup so the bay is known-closed. Safe here, '
         'unlike in flight -- but off by default so nothing moves unasked.'),

        # ---- the lights ----
        ('led', 'true', 'Run the WS2812B node.'),
        ('num_pixels', '5', ''),
        ('blink_hz', '2.0', ''),
    ]
    declared = [DeclareLaunchArgument(n, default_value=d, description=h)
                for n, d, h in args]

    # release_enabled:=false must actually mean it, so the release node does
    # not run at all rather than running and being asked not to fire.
    servo_armed = PythonExpression(
        ["'", L('release_enabled'), "'.lower() not in ('false', '0')"])

    microxrce = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_xrce_dds_agent', output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
        condition=IfCondition(L('agent')),
    )

    sensor = Node(
        package='drone_testing', executable='thermal_sensor',
        name='thermal_sensor', output='screen', emulate_tty=True,
        parameters=[{'refresh_hz': L('refresh_hz'),
                     'publish_preview': L('publish_preview'),
                     'stream_port': L('stream_port'),
                     'jpeg_quality': L('jpeg_quality'),
                     # The boxes on the stream must be the boxes the bench
                     # node acts on, or you are watching a second opinion.
                     'min_contrast': L('min_contrast'),
                     'min_blob_pixels': L('min_blob_pixels')}],
    )

    bench = Node(
        package='drone_testing', executable='thermal_bench',
        name='thermal_bench', output='screen', emulate_tty=True,
        parameters=[{'min_contrast': L('min_contrast'),
                     'min_blob_pixels': L('min_blob_pixels'),
                     'hfov_deg': L('hfov_deg'),
                     'vfov_deg': L('vfov_deg'),
                     'cam_yaw_deg': L('cam_yaw_deg'),
                     'flip_lr': L('flip_lr'),
                     'flip_ud': L('flip_ud'),
                     'drop_altitude': L('drop_altitude'),
                     'altitude_tolerance': L('altitude_tolerance'),
                     'centre_tolerance': L('centre_tolerance'),
                     'settle_seconds': L('settle_seconds'),
                     'require_altitude': L('require_altitude'),
                     'lost_seconds': L('lost_seconds'),
                     'rearm_seconds': L('rearm_seconds'),
                     'servo_hold_seconds': L('servo_hold_seconds'),
                     'drop_trigger_topic': L('drop_trigger_topic')}],
    )

    servo = Node(
        package='drone_testing', executable='servo_controller',
        name='servo_controller', output='screen', emulate_tty=True,
        parameters=[{'drop_trigger_topic': L('drop_trigger_topic'),
                     'servo_index': L('servo_index'),
                     'servo_drop_value': L('servo_drop_value'),
                     'servo_neutral_value': L('servo_neutral_value'),
                     'servo_hold_seconds': L('servo_hold_seconds'),
                     'servo_command': L('servo_command'),
                     'servo_function': L('servo_function'),
                     'close_on_start': L('servo_close_on_start')}],
        condition=IfCondition(servo_armed),
    )

    led = Node(
        package='drone_testing', executable='led_status', name='led_status',
        output='screen', emulate_tty=True,
        parameters=[{'num_pixels': L('num_pixels'), 'blink_hz': L('blink_hz')}],
        condition=IfCondition(L('led')),
    )

    return LaunchDescription(declared + [microxrce, sensor, bench, servo, led])
