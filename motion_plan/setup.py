#!/usr/bin/env python3

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['roadmap_sampler'],
    package_dir={'': 'roadmap_sampler/src'},
)

setup(**d)
