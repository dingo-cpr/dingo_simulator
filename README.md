# dingo_simulator
This is the ROS 2 simulation package for the Dingo robot

## Installation
You will also need the Velodyne description package.
You can grab it using:

```
git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git
cd velodyne_simulator
git checkout gonzalodepedro/foxy-devel
```

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
In order to use the simulation to its fullest potential, you will need to run two launch files.
```
ros2 launch dingo_gazebo dingo_world.launch.py
```
Wait for Gazebo to fully start. It takes the velodyne plugin 30/60 seconds to start. Once Gazebo is started and you can see the robot launch the next file.
```
ros2 launch dingo_gazebo start_dingo_controls.launch.py
```
