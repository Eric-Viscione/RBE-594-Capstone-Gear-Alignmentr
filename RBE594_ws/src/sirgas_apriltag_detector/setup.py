from setuptools import setup

package_name = 'sirgas_apriltag_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/black_tag_detector.launch.py',
                                                'launch/apriltag_detector.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamar',
    maintainer_email='',
    description='AprilTag and black-square pose estimation',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector = sirgas_apriltag_detector.apriltag_detector:main',
            'black_tag_detector = sirgas_apriltag_detector.black_tag_detector:main',
        ],
    },
)
