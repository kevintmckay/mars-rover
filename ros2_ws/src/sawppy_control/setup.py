from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sawppy_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin',
    maintainer_email='kevin@localhost',
    description='Ackermann kinematics and cmd_vel controller for Sawppy rover',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_controller = sawppy_control.ackermann_controller:main',
            'odometry_node = sawppy_control.odometry_node:main',
        ],
    },
)
