from setuptools import find_packages
from setuptools import setup

setup(
    name='moveit',
    version='2.12.3',
    packages=find_packages(
        include=('moveit', 'moveit.*')),
)
