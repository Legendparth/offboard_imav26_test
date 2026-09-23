"""
Bench arm test launch. No ZED, no localization -- fewest moving parts.

The uXRCE-DDS agent is not started here: imav_bringup's bringup.launch.py
runs it. The node only streams the offboard heartbeat; you switch to Offboard
and arm from the TX.

NOTE: launching the mission node this way means stdin is not a tty, so the
'q' keyboard abort will be DISABLED. For the first arm tests, leave
agent_only:=true (the default, which starts nothing) and run the mission node
by hand in a second tmux pane so you keep the abort key.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    agent_only = LaunchConfiguration('agent_only')

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
            description='Do not start the mission node; run it manually.'),
        mission_node,
    ])