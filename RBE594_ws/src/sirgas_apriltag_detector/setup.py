from setuptools import setup
import os
from glob import glob

package_name = 'sirgas_apriltag_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamar',
    maintainer_email='tboone@wpi.edu',
    description='AprilTag detector node (Python) for SIRGAS project.',
    license='TODO',
    entry_points={
        'console_scripts': [
            # ros2 run sirgas_apriltag_detector apriltag_detector
            'apriltag_detector = sirgas_apriltag_detector.apriltag_detector:main',
        ],
    },
)
