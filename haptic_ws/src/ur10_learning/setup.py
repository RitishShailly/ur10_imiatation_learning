from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur10_learning'

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
            'ur10_teleop_joystick = ur10_learning.ur10_teleop_joystick:main',
            'play_trajectory_node = ur10_learning.play_trajectory:main',
            'record_demo = ur10_learning.record_demo:main',
            'play_model = ur10_learning.play_model:main',
            'play_model2 = ur10_learning.play_model2:main',
            'trainDAgger = ur10_learning.trainDAgger:main',
            'test_DAgger_model = ur10_learning.test_DAgger_model:main',
            'test_fall21_model = ur10_learning.test_fall21_model:main',
            'test_segment_model = ur10_learning.test_segment_model:main'
        ],
    },
)
