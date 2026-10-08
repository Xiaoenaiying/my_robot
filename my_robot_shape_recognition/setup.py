import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_shape_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 把 launch 目录安装到 install/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 把 config 目录安装到 install/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='OpenCV shape recognition node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 这一行最关键：注册可执行命令
            'shape_recognizer_node = my_robot_shape_recognition.shape_recognizer_node:main',
        ],
    },
)