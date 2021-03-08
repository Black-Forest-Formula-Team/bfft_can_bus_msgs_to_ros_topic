#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     packages=['bfft_CAN_msgs_to_ROS_topic', 'bfft_CAN_msgs_to_ROS_topic.utilities_IMU'],
     package_dir={'': 'src'}
)

setup(**setup_args)