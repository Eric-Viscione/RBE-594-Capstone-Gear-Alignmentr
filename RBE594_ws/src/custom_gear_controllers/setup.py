from setuptools import setup
from glob import glob
import os

package_name = 'custom_gear_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the plugin description XML file
        (os.path.join('share', package_name), glob('resource/*.xml')), 
        # Install launch/config files if any
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'rclpy', 'controller_interface'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Custom ROS 2 Controller for enforcing gear ratio constraints.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
        # --- CRITICAL: ROS 2 CONTROL PLUGIN ENTRY POINT ---
        'controller_interface.controllers': [
            'GearConstraintController = custom_gear_controllers.gear_constraint_controller:GearConstraintController',
        ],
    },
)