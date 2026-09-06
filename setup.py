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
            'pixhawk_node = drone_testing.pixhawk_node:main',
            'offboard_mission = drone_testing.offboard_mission:main',
            'offboard_takeoff = drone_testing.offboard_takeoff:main',
            'offboard_translate = drone_testing.offboard_translate:main',
            'offboard_sequence = drone_testing.offboard_sequence:main',
            'lcd_status = drone_testing.lcd_status:main',
            'zed_localization = drone_testing.zed_localization:main',
            'cam = drone_testing.cam:main',
            'fc_reboot = drone_testing.fc_reboot:main',
            'window_detect = drone_testing.window_detect:main',
            'window_scan = drone_testing.window_scan:main',
        ],
    },
)
