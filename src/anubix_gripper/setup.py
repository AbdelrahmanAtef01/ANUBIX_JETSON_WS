from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_gripper'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='anubix@example.com',
    description='ANUBIX Gripper Control — myGripperF100 over USB-RS485',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gripper_node = anubix_gripper.gripper_node:main',
            'gripper_sender = anubix_gripper.gripper_sender:main',
        ],
    },
)
