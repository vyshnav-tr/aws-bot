from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'mission_manager'

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
            'battery_node = mission_manager.battery_node:main',
            'auto_dock = mission_manager.auto_docking_node:main',
            'follow_robot = mission_manager.final_person_follower:main',

        ],
    },
)
