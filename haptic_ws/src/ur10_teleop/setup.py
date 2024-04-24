from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur10_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*.urdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rshailly',
    maintainer_email='rshailly@vt.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur10_teleop_keyboard = ur10_teleop.ur10_teleop_keyboard:main',
            'ur10_teleop_joystick = ur10_teleop.ur10_teleop_joystick:main',
            'fetch_urdf = ur10_teleop.robot_description_fetcher:main',
            'fetch_joint_positions = ur10_teleop.get_joint_positions:main',
            'get_fk = ur10_teleop.get_fk:main',
            'play_trajectory_node = ur10_teleop.play_trajectory:main'
        ],
    },
)
