#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['battery_mockup', 'battery_mockup_ros'],
 package_dir={'battery_mockup': 'common/src/battery_mockup', 'battery_mockup_ros': 'ros/src/battery_mockup_ros'}
)

setup(**d)
