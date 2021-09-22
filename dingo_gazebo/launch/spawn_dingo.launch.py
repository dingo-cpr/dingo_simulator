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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	description_path = os.path.join(get_package_share_directory('dingo_description'), 'launch', 'description.launch.py')

	# Launch configuration variables specific to simulation
	x_pose = LaunchConfiguration('x_pose')
	y_pose = LaunchConfiguration('y_pose')
	z_pose = LaunchConfiguration('z_pose')
	yaw = LaunchConfiguration('yaw')
	config = LaunchConfiguration('config')

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

	# Specify the actions
	description_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(description_path),
		launch_arguments={'config' : config, 'physical_robot' : 'false'}.items()
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

	start_dingo_control = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(os.path.join(
			get_package_share_directory('dingo_control'), 'launch', 'control.launch.py')),
		launch_arguments={'physical_robot' : 'false'}.items()
	)

	spawn_velocity_controller = ExecuteProcess(
		cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
			'simulation_velocity_controller'],
		output='screen'
	)

	load_joint_trajectory_controller = ExecuteProcess(
		cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
			'joint_trajectory_controller'],
		output='screen'
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
	ld.add_action(start_dingo_control)
	ld.add_action(spawn_velocity_controller)

	return ld