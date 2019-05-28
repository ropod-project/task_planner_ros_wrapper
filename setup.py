#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['task_planner_ros_wrapper', 'task_planner_ros_utils'],
   package_dir={'task_planner_ros_wrapper': 'ros/src/task_planner_ros_wrapper',
                'task_planner_ros_utils': 'ros/src/task_planner_ros_utils'}
)

setup(**d)
