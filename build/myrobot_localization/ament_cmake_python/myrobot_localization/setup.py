from setuptools import find_packages
from setuptools import setup

setup(
    name='myrobot_localization',
    version='0.0.0',
    packages=find_packages(
        include=('myrobot_localization', 'myrobot_localization.*')),
)
