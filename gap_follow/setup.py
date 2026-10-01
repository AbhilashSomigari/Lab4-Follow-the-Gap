from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'gap_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Lab 4: F1TENTH Follow the Gap reactive obstacle avoidance.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gap_follow_node = gap_follow.gap_follow_node:main',
        ],
    },
)
