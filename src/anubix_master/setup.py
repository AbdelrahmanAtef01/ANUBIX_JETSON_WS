from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_master'

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
    install_requires=['setuptools', 'omnilink>=0.6.0'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='anubix@example.com',
    description='ANUBIX ROS 2 Master Node - OmniLink to 4-stack bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'master_node = anubix_master.ros_master_node:main',
        ],
    },
)
