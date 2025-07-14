from setuptools import find_packages
from setuptools import setup

setup(
    name='myamr_localization',
    version='0.0.0',
    packages=find_packages(
        include=('myamr_localization', 'myamr_localization.*')),
)
