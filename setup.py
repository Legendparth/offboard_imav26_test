from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'systemd'), glob(os.path.join(package_name, '*.service')))
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='ark-jetson-orin',
    maintainer_email='ishan.aphanse2807@gmail.com',
    description='Pixhawk telemetry reader and offboard launch',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'servo_controller = drone_testing.servo_controller:main',
            'pixhawk_node = drone_testing.pixhawk_node:main',
            'offboard_mission = drone_testing.offboard_mission:main',
            'offboard_takeoff = drone_testing.offboard_takeoff:main',
            'offboard_translate = drone_testing.offboard_translate:main',
            'offboard_sequence = drone_testing.offboard_sequence:main',
            'offboard_sequence_vio = drone_testing.offboard_sequence_vio:main',
            'lcd_status = drone_testing.lcd_status:main',
            'zed_localization = drone_testing.zed_localization:main',
            'cam = drone_testing.cam:main',
            'fc_reboot = drone_testing.fc_reboot:main',
            'window_detect = drone_testing.window_detect:main',
            'window_scan = drone_testing.window_scan:main',
            'window_traverse = drone_testing.window_traverse:main',
            'bar_detect = drone_testing.bar_detect:main',
            'bar_down_check = drone_testing.bar_down_check:main',
            'bar_cross = drone_testing.bar_cross:main',
            'course_fsm = drone_testing.course_fsm:main',
            'rng_dropout = drone_testing.rng_dropout:main',
            'tube_detect = drone_testing.tube_detect:main',
            'tube_cross = drone_testing.tube_cross:main',
            'aruco_pose = drone_testing.aruco_pose:main',
            'precision_land = drone_testing.precision_land:main',
            'led_status = drone_testing.led_status:main',
            'servo_test = drone_testing.servo_test:main',
            'thermal_sensor = drone_testing.thermal_sensor:main',
            'thermal_drop = drone_testing.thermal_drop:main',
            'thermal_fsm = drone_testing.thermal_fsm:main',
            'thermal_sim = drone_testing.thermal_sim:main',
            'thermal_bench = drone_testing.thermal_bench:main',
            'servo_control = drone_testing.servo_control:main'
        ],
    },
)
