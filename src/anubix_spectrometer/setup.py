from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_spectrometer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='anubix@example.com',
    description='ANUBIX Spectrometer Stack - spectral data acquisition and ML analysis',
    license='MIT',
    entry_points={
        'console_scripts': [
            'spectrometer_node = anubix_spectrometer.spectrometer_node:main',
        ],
    },
)
