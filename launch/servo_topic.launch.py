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

    servo_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_testing', 
                executable='servo_controller',
                name='servo_drop_test',
                output='screen',
                emulate_tty=True,
            )
        ],
        condition=UnlessCondition(agent_only),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_only', default_value='false',
            description='Start only the uXRCE-DDS agent if true.'),
        microxrce_node,
        servo_node,
    ])