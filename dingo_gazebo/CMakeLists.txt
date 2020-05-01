cmake_minimum_required(VERSION 2.8.3)
project(dingo_gazebo)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()

roslaunch_add_file_check(launch/dingo_d_world.launch)
roslaunch_add_file_check(launch/dingo_o_world.launch)

install(DIRECTORY launch Media worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
