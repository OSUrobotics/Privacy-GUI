#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['privacy_zones'],
    package_dir={'': 'src'},
    install_requires=['rospkg'],
    )

setup(**d)
