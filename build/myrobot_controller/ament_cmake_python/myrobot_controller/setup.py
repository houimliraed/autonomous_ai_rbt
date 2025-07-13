from setuptools import find_packages
from setuptools import setup

setup(
    name='myrobot_controller',
    version='0.0.0',
    packages=find_packages(
        include=('myrobot_controller', 'myrobot_controller.*')),
)
