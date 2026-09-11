import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wheel_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='thienductang@gmail.com',
    description='Two-wheel differential base controller with yaw-rate and heading control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_control_node = wheel_control.wheel_control_node:main',
        ],
    },
)
