from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/turtlebot/launch', ['launch/turtlebot_launch.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oorischubert',
    maintaiemail='oorischubert@gmail.com',
    description='my first robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = turtlebot.motor_controller:main',
            'teleop = turtlebot.teleop:main',
        ],
    },
)
