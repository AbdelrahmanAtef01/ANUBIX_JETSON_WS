from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='team@anubix.io',
    description='ANUBIX vision stack — YOLO leaf detection on Jetson Orin Nano',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = anubix_vision.vision_node:main',
        ],
    },
)
