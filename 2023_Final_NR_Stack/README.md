# Final_NR_Stack

## Preliminary
1. Follow the instructions ([here](https://docs.px4.io/main/en/ros/ros2_comm.html)) to set up the PX4 Autopilot Stack, ROS2, Micro XRCE-DDS Agent & Client, and build a ROS2 workspace with the necessary px4 communication repos
2. In the same workspace with the communication folders as above, go to the /src/ folder and clone this repository
3. In the root of the workspace, build everything as:
```
colcon build
```
5. Create a conda environment with simpy, scipy, and numpy

## Simulation Run and Communication
1. Run the SITL simulation
```
make px4_sitl gazebo-classic
```
2. Run the Micro XRCE Agent as the bridge between ROS2 and PX4.
```
MicroXRCEAgent udp4 -p 8888
```


## To Run The NR Controller Computation and Offboard Publisher
The reference path may be changed through the reffunc variable starting in line 139 of the nr_tracker_final.py file. New ones may be defined and referenced here.
1. If changes have been made to the reference, in one terminal, go to the root of your ROS2 workspace and build this package:
```
colcon build --packages-select Final_NR_Wardi_Tracker_Stack
```

### Through launch file:
1. In another terminal tab, source the environment from the root of your ROS2 workspace: 
```
source install/setup.bash
```
2. Activate your conda environment in this same terminal with sourcing
3. After sourcing and activating environment, run the launch file:
```
ros2 launch Final_NR_Wardi_Tracker_Stack nr_tracker_launch.py
```

### This can also be run by running the two separate files on their own:
1. In one terminal tab, source the environment as shown before, activate the environment and run:
```
ros2 run Final_NR_Wardi_Tracker_Stack nr_tracker_final.py
```
This is the computation file

2. In another terminal, source again, and run:
```
ros2 run Final_NR_Wardi_Tracker_Stack offboard_pub
```
This is the offboard publisher that takes the computed input and communicates it to the robot
