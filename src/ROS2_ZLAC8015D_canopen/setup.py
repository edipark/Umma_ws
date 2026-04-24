from setuptools import setup
import os
from glob import glob

package_name = 'zlac8015d_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 control node for ZLAC8015D CANopen servo hub motor driver',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'zlac8015d_control_node = zlac8015d_control.zlac8015d_control_node:main',
        ],
    },
)
