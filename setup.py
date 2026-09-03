from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tb3_multi_robot'

setup(
    name=package_name,
    version='2.2.6',
    packages=find_packages(),
    data_files=[
        (f'share/{package_name}', ['package.xml']),
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'launch/nav2_bringup'),
            glob('launch/nav2_bringup/*.py'),
        ),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (
            os.path.join('share', package_name, 'models/turtlebot3_burger'),
            glob('models/turtlebot3_burger/*'),
        ),
        (
            os.path.join('share', package_name, 'models/turtlebot3_waffle'),
            glob('models/turtlebot3_waffle/*'),
        ),
        (os.path.join('share', package_name, 'models/turtlebot3_waffle_pi'),
         glob('models/turtlebot3_waffle_pi/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
        (
            os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Arshad Mehmood',
    maintainer_email='arshadm78@yahoo.com',
    description='Multi-robot simulation using TurtleBot3 and Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'turtlebot3_drive = multi_robot_scripts.turtlebot3_drive:main',
            'chase_tag = multi_robot_scripts.chase_tag:main',
            'tb3_follow_tb1 = multi_robot_scripts.tb3_follow_tb1:main',
            'tf_pose_publisher = multi_robot_scripts.tf_pose_publisher:main',
            (
                'simulation_health_monitor = '
                'multi_robot_scripts.simulation_health_monitor:main'
            ),
        ],
    },
)
