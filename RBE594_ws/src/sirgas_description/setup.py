from setuptools import setup
import os
from glob import glob

package_name = 'sirgas_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],                         # no package dir
    py_modules=['apriltag_detector'],    # single module file
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][y|m]'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*'))),
        (os.path.join('share', package_name, 'urdf', 'inc'), glob(os.path.join('urdf', 'inc', '*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'meshes', 'sim_cam'), glob(os.path.join('meshes', 'sim_cam', '*'))),
        (os.path.join('share', package_name, 'meshes', 'gripper_meshes'), glob(os.path.join('meshes', 'gripper_meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='AprilTag detector and robot descriptions.',
    license='TODO',
    entry_points={
        'console_scripts': [
            # left side: executable name for `ros2 run`
            # right side: module:function that has `main()`
            'apriltag_detector = apriltag_detector:main',
        ],
    },
)
