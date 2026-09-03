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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
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
            'zed_localization = drone_testing.zed_localization:main',
            'cam = drone_testing.cam:main',
        ],
    },
)
