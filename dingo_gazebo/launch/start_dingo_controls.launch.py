# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

	# Declare the launch arguments
	start_dingo_control = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(
			get_package_share_directory('dingo_control'), 'launch', 'control.launch.py')),
		launch_arguments={
			'physical_robot' : 'false',
			'use_sim_time' : 'true',
		}.items()
	)

	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(start_dingo_control)

	return ld