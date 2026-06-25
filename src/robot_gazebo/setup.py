from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'robot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vyshnav-si3514',
    maintainer_email='vyshnav-si3514@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'docking_node = robot_gazebo.docking_bridge:main',
            'aruco_pose_bridge = robot_gazebo.aruco_pose_bridge:main',
            'route_navigator = robot_gazebo.route_navigator:main',
        ],
    },
)
