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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	description_path = os.path.join(
		get_package_share_directory('dingo_description'), 'launch', 'description.launch.py')
	dingo_controls_path = os.path.join(
		get_package_share_directory('dingo_gazebo'), 'launch', 'start_dingo_controls.launch.py')
	# Launch configuration variables specific to simulation
	x_pose = LaunchConfiguration('x_pose')
	y_pose = LaunchConfiguration('y_pose')
	z_pose = LaunchConfiguration('z_pose')
	yaw = LaunchConfiguration('yaw')
	config = LaunchConfiguration('config')
	use_sim_time = LaunchConfiguration('use_sim_time')

	# Declare the launch arguments
	declare_use_sim_time_cmd = DeclareLaunchArgument(
		'use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true')

	declare_x_position_cmd = DeclareLaunchArgument(
		'x_pose', default_value='0.0',
		description='x_pose')

	declare_y_position_cmd = DeclareLaunchArgument(
		'y_pose', default_value='0.0',
		description='y_pose')

	declare_z_position_cmd = DeclareLaunchArgument(
		'z_pose', default_value='0.1',
		description='z_pose')

	declare_yaw_cmd = DeclareLaunchArgument(
		'yaw', default_value='0.0',
		description='yaw')

	declare_config_cmd = DeclareLaunchArgument(
		'config',
		default_value=os.getenv('DINGO_CONFIG', 'base'),
		description='get the dingo configuration')

	dingo_control_directory = get_package_share_directory('dingo_control')

	if os.getenv('DINGO_OMNI', 0):
		control_yaml = os.path.join(dingo_control_directory, 'config', 'control_omni.yaml')
	else:
		control_yaml = os.path.join(dingo_control_directory, 'config', 'control_diff.yaml')

	# Rewrite config file
	param_substitutions = {
		'use_sim_time': use_sim_time,
	}

	rewritten_control_yaml = RewrittenYaml(
		source_file=control_yaml,
		root_key='',
		param_rewrites=param_substitutions,
		convert_types=True)

	# Specify the actions
	description_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(description_path),
		launch_arguments={'config' : config, 'physical_robot' : 'false', 'control_yaml_file' : rewritten_control_yaml}.items()
	)

	start_gazebo_ros_spawner_cmd = 	Node(
		package='gazebo_ros', 
		executable='spawn_entity.py',
		arguments=['-entity', 'dingo',
								'-topic', 'robot_description',
                '-x', x_pose,
                '-y', y_pose,
								'-z', z_pose
							],
		output='screen',
	)

	delayed_dingo_controls_cmd = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=start_gazebo_ros_spawner_cmd,
			on_exit=[
				IncludeLaunchDescription(
					PythonLaunchDescriptionSource(dingo_controls_path)
				)			
			]
		)
	)

	ld = LaunchDescription()

	# Declare the launch options
	ld.add_action(declare_use_sim_time_cmd)
	ld.add_action(declare_x_position_cmd)
	ld.add_action(declare_y_position_cmd)
	ld.add_action(declare_z_position_cmd)
	ld.add_action(declare_yaw_cmd)
	ld.add_action(declare_config_cmd)

	# Add any conditioned actions
	ld.add_action(description_cmd)
	ld.add_action(start_gazebo_ros_spawner_cmd)
	ld.add_action(delayed_dingo_controls_cmd)

	return ld