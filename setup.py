#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dir = generate_distutils_setup(
    packages=['tele_dir'],
    package_dir={'' : 'src'},
)

setup(**dir)
