# dingo_simulator
This is the ROS 2 simulation package for the Dingo robot

## Installation
Make sure you run rosdep from your workspace folder.
```
rosdep install --from-paths src --ignore-src -r -y
```

Then simply build the packages using colcon build
```
colcon build --symlink-install
```

Finally, add the following line to your .bashrc file
```
export DINGO_LASER_3D 1
```

## Using
In order to use the simulation, you will need to run the following launch file.
```
ros2 launch dingo_gazebo dingo_world.launch.py
```
Wait for Gazebo to fully start.
The Velodyne simulator can take up to 30 seconds to start, meaning the gazebo_ros spawner will take that long.
As a result, ros2_control is delayed by about 20 seconds.
This will ensure that the controllers don't time out while waiting to connect to the server started in the urdf.
