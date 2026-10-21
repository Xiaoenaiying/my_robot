import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_yolo_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,'utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 把下面这一行加进去，把 models 文件夹整个安装到 install/share 目录下
        (os.path.join('share', package_name, 'models'), glob('models/*')),
        # 把 my_robot_checkself 文件夹也安装过去
        (os.path.join('share', package_name, 'my_yolo_checkself'), glob('my_yolo_checkself/*')),
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