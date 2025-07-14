from setuptools import find_packages
from setuptools import setup

setup(
    name='myamr_controller',
    version='0.0.0',
    packages=find_packages(
        include=('myamr_controller', 'myamr_controller.*')),
)
