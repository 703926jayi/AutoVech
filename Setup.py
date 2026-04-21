# Once the repository is reorganized, this file should live inside:
# autovech/ros2_ws/src/autovech/setup.py
#
# To build the workspace and test the nodes:
# 1. cd ~/autovech/ros2_ws
# 2. colcon build --packages-select autovech
# 3. source install/setup.bash

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autovech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #Add more executables here once created in form 'executable_name = autovech.file:start_function'
            'throttle_executable = autovech.Throttle:start_throttle',
            'brakes_executable = autovech.Brakes:start_brakes',
            'gamepad_executable = autovech.Gamepad:gamepad_start',
            'steering_executable = autovech.Steering:steering_start',
        ],
    },
)
