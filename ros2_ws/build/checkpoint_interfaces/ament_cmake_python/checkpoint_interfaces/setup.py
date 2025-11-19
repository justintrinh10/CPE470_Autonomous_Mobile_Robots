from setuptools import find_packages
from setuptools import setup

setup(
    name='checkpoint_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('checkpoint_interfaces', 'checkpoint_interfaces.*')),
)
