import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mega_bridge'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='thienductang@gmail.com',
    description='Serial bridge to the Arduino Mega: wheels, gantry steppers, gripper.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mega_bridge_node = mega_bridge.mega_bridge_node:main',
        ],
    },
)
