import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='YOLO detection node',
    license='TODO',
    entry_points={
        'console_scripts': [
            'yolo_detector_node = my_robot_yolo_detection.yolo_detector_node:main',
        ],
    },
)