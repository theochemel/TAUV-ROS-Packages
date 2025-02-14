# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['thruster_managers',
              'thruster_managers.models',
              'controllers',
              'planners',
              'planners.keyboard_planner',
              'planners.teleop_planner',
              'planners.trajectory_planner',
              'state_estimation.depth_odometry',
              'state_estimation.twist_transformer',
              'state_estimation.pose_graph',
              'teleop',
              'vision',
              'vision.detector_bucket',
              'vision.detectors',
              'dynamics',
              'tauv_util',
              'motionlib'],
    # packages=find_packages(),
    package_dir={'': 'src'},
    requires=['rospy'],
    scripts=['scripts/thruster_allocator',
             'scripts/keyboard_planner',
             'scripts/teleop_planner',
             'scripts/depth_odometry',
             'scripts/twist_transformer',
             'scripts/pose_graph',
             'scripts/detector_bucket',
             'scripts/attitude_controller',
             'scripts/mpc_traj_follower',
             'scripts/cnn_detector',
             'scripts/gate_detector'],

)
setup(**setup_args)
