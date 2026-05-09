from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anubix_supabase'

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
    install_requires=['setuptools', 'supabase'],
    zip_safe=True,
    maintainer='ANUBIX Team',
    maintainer_email='anubix@example.com',
    description='ANUBIX Supabase Uploader — uploads spectral analysis results to Supabase',
    license='MIT',
    entry_points={
        'console_scripts': [
            'supabase_node = anubix_supabase.supabase_node:main',
        ],
    },
)
