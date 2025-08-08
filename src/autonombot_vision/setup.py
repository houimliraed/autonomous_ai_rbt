from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonombot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ray',
    maintainer_email='houimliraed@outlook.fr',
    description='YOLO v8 object detection for Autonombot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector = autonombot_vision.yolo_detector:main',
            'vision_obstacle_detector = autonombot_vision.vision_obstacle_detector:main',
            'vision_costmap_updater = autonombot_vision.vision_costmap_updater:main',
            'minimal_yolo_obstacles = autonombot_vision.minimal_yolo_obstacles:main',
            'advanced_vision_navigator = autonombot_vision.advanced_vision_navigator:main',
            'performance_monitor = autonombot_vision.performance_monitor:main',
        ],
    },
)
