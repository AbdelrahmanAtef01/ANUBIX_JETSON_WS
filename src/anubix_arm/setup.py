from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='anubix@example.com',
    description='ANUBIX Arm Control Stack - manipulator and gripper',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_node = anubix_arm.arm_node:main',
            'mission = anubix_arm.mission:main',
        ],
    },
)
