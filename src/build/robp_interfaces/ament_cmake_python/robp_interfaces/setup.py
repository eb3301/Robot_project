from setuptools import find_packages
from setuptools import setup

setup(
    name='robp_interfaces',
    version='1.0.0',
    packages=find_packages(
        include=('robp_interfaces', 'robp_interfaces.*')),
)
