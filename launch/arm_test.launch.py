"""
Bench arm test launch. No ZED, no localization -- fewest moving parts.

NOTE: launching the mission node this way means stdin is not a tty, so the
'q' keyboard abort will be DISABLED. For the first arm tests, launch only
the agent (agent_only:=true) and run the mission node by hand in a second
tmux pane so you keep the abort key.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600'],
    )

    # Delay so the DDS session is up and PX4 topics exist before the node
    # starts publishing. Without this the first setpoints are dropped.
    mission_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing',
                executable='offboard_mission',
                name='arm_disarm_test',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='true',
            description='Start only the uXRCE-DDS agent; run the mission node manually.'),
        microxrce_node,
        mission_node,
    ])