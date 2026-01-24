#!/usr/bin/env python3
"""
Setup script for aae4011_ai_uas ROS package.
This enables Python module imports within the package.
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['aae4011_ai_uas'],
    package_dir={'': 'src'},
    requires=['rospy', 'std_msgs', 'numpy', 'matplotlib', 'pandas', 'scipy']
)

setup(**setup_args)
