from setuptools import find_packages
from setuptools import setup

setup(
    name='behavior_interface',
    version='0.0.0',
    packages=find_packages(
        include=('behavior_interface', 'behavior_interface.*')),
)
