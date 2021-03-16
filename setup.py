#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['bfft_can_bus_msgs_to_ros_topic', 'bfft_can_bus_msgs_to_ros_topic.utilities_IMU'],
     package_dir={'': 'src'}
)

setup(**setup_args)