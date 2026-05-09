from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_jetson_bridge'

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
    description='ANUBIX Jetson Bridge - cross-machine link monitor and traffic logger',
    license='MIT',
    entry_points={
        'console_scripts': [
            'jetson_bridge_node = anubix_jetson_bridge.jetson_bridge_node:main',
        ],
    },
)
