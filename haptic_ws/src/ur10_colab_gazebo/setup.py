from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur10_colab_gazebo'

def get_data_files():
    data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ]
    models_dir = os.path.join('models', 'gazebo_models')
    for root, dirs, files in os.walk(models_dir):
        for file in files:
            source_file = os.path.relpath(os.path.join(root, file), start=models_dir)
            destination = os.path.join('share', package_name, models_dir, os.path.dirname(source_file))
            data_files.append((destination, [os.path.join(root, file)]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rshailly',
    maintainer_email='rshailly@vt.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'add_simple_env_objects = ur10_colab_gazebo.add_simple_env_objects:main',
            'del_simple_env_objects = ur10_colab_gazebo.del_simple_env_objects:main',
        ],
    },
)
