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
It takes 20 seconds for the ros2 controls to start (it uses a launch delay, to avoid timing out).
