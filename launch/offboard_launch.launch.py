import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_name = 'drone_testing'
    zed_wrapper_dir = get_package_share_directory('zed_wrapper')
    zed_launch_file = os.path.join(
        zed_wrapper_dir, 'launch', 'zed_camera.launch.py'
    )

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_file),
        launch_arguments={
            'camera_model': 'zed',  # zed, zedm, zed2, zed2i, zedx
            'rviz': 'false',  # Set to 'true' if you want RViz
            # Optional extra parameters:
            # 'camera_name': 'zed',
            # 'node_name': 'zed_node',
            # 'publish_tf': 'true',
        }.items(),
    )

    zed_localization_node = Node(
        package='drone_testing',
        executable='zed_localization',
        name='zed_localization_node',
        output='screen'
    )

    offboard_mission = Node(
        package='drone_testing',
        executable='offboard_mission',
        name='offboard_mission_node',
        output='screen'
    )


    return LaunchDescription([
        zed_camera_launch,
        offboard_mission,
        # zed_localization_node

    ])
