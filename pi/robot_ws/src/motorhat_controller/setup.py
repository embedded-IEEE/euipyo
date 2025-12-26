from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'motorhat_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='MotorHAT controller for ROS2 Humble',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch ÆÄÀÏ ¼³Ä¡
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'motor_node = motorhat_controller.motor_node:main',
        ],
    },
)

