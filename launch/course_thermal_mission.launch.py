"""
Course, then thermal drop, in one flight. See drone_testing/mission_fsm.py.

    course_mission.launch.py   handoff:=true  -> mission_course
    thermal_drop.launch.py     handoff:=true  -> mission_thermal (silent until
                                                 the course hands over)

WHAT TO RUN

    ros2 launch drone_testing course_thermal_mission.launch.py
    ros2 run drone_testing mission_course --ros-args -p takeoff_altitude:=1.2 ...

or all in one, RC kill switch only (no q/k):

    ros2 launch drone_testing course_thermal_mission.launch.py agent_only:=false

agent_only only affects the COURSE node (the one you fly from the terminal).
mission_thermal is always launched: it publishes nothing until the handoff,
and if it never acks, the course finishes its own way after handoff_timeout.

EVERY OTHER ARGUMENT belongs to one of the two files and is passed through
with a prefix, because the two share names that mean different things
(stream_port, takeoff_altitude, flight_node_delay, led, ...):

    course.<arg>:=<value>    -> course_mission.launch.py
    thermal.<arg>:=<value>   -> thermal_drop.launch.py

e.g.  course.window_after_tubes:=true  thermal.survey_altitude:=2.3

THE MARKER LOOK AFTER EACH STEP RIGHT is top-level, no prefix:

    land_look_wait / land_look_climb / land_look_forward      (thermal)
    pad_look_wait  / pad_look_climb  / pad_look_forward       (course)
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _prefixed(context, prefix):
    """course.x:=1 on the command line -> {'x': '1'}."""
    return {k[len(prefix):]: v for k, v in context.launch_configurations.items()
            if k.startswith(prefix)}


# The marker look after each right offset (see PAD_LOOK in course_fsm.py and
# LAND_LOOK in thermal_fsm.py): top-level here because they are the ones
# worth tuning for this mission. course./thermal.-prefixed values still win.
LOOK_ARGS = {
    'course': [
        ('pad_look_wait', '1.0',
         's looking for the guide marker after the course\'s step right, '
         'before climbing and creeping forward. Only flown if the handoff '
         'fails and the course finishes on its own pad.'),
        ('pad_look_climb', '0.30', 'm climbed for that look.'),
        ('pad_look_forward', '0.50', 'm crept forward for it. Capped at 0.5.'),
    ],
    'thermal': [
        ('land_look_wait', '1.0',
         's looking for the landing marker after the post-drop step right, '
         'before climbing and creeping forward.'),
        ('land_look_climb', '0.30', 'm climbed for that look.'),
        ('land_look_forward', '0.50', 'm crept forward for it. Capped at 0.5.'),
    ],
}


def _top(context, args):
    return {n: LaunchConfiguration(n).perform(context) for n, _, _ in args}


def _includes(context):
    launch_dir = PathJoinSubstitution([FindPackageShare('drone_testing'), 'launch'])

    course_args = {
        'handoff': 'true',
        'agent_only': LaunchConfiguration('agent_only').perform(context),
        # thermal_drop's aruco_pose owns the down camera and :8083; the
        # course never reaches its own pad stages in this mission.
        'pad_detector': 'false',
        # :8082 is the thermal camera's stream.
        'tube_stream_port': '8085',
    }
    course_args.update(_top(context, LOOK_ARGS['course']))
    course_args.update(_prefixed(context, 'course.'))

    thermal_args = {
        'handoff': 'true',
        'fsm': 'true',
        'agent_only': 'false',
        'mode': 'fly',
        'handoff_start': LaunchConfiguration('handoff_start').perform(context),
    }
    thermal_args.update(_top(context, LOOK_ARGS['thermal']))
    thermal_args.update(_prefixed(context, 'thermal.'))

    def include(name, args):
        return GroupAction(scoped=True, forwarding=False, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, name])),
                launch_arguments=list(args.items()))])

    return [include('course_mission.launch.py', course_args),
            include('thermal_drop.launch.py', thermal_args)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='true = run mission_course by hand (keeps q/k). '
                        'false = launch it too.'),
        DeclareLaunchArgument(
            'handoff_start', default_value='survey',
            description='survey = climb and look for the boxes where the '
                        'course ended. marker = thermal_fsm\'s marker hunt '
                        'first.'),
    ] + [DeclareLaunchArgument(n, default_value=d, description=h)
         for side in ('course', 'thermal') for n, d, h in LOOK_ARGS[side]] + [
        OpaqueFunction(function=_includes),
    ])
