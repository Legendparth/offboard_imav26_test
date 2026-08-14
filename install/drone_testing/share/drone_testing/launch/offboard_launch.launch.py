import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'drone_testing'

    microxrce_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_xrce_dds_agent',
        output='screen',
        arguments=['serial', '--dev', '/dev/ttyTHS1', '-b', '921600']
    )


    #pixhawk_comm_node = Node(
     #   package=pkg_name, 
      #  executable='pixhawk_node',
       # name='pixhawk_reader',
        #output='screen',
     #   parameters=[
      #      {'port': '/dev/ttyTHS1'},
       #     {'baud': 57600},
        #    {'rate': 4}
   #     ]
    #)

    offboard_mission_node = Node(
        package=pkg_name,
        executable='offboard_mission',
        name='offboard_mission_node',
        output='screen'
    )

    return LaunchDescription([
        microxrce_node,
     #   pixhawk_comm_node,
        offboard_mission_node
    ])
