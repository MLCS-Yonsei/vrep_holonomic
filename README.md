# vrep_holonomic

ROS V-rep simulation package for Mechanum-wheeled mobile robot

<br/>

## Requirements

### ROS
http://wiki.ros.org/ROS/Installation

### v-rep
http://www.coppeliarobotics.com/downloads.html

### CasADi
This package uses the python 2.7  version of Casadi
https://web.casadi.org/get/

### ros-navigation & gmapping

- ROS-melodic
```bash
sudo apt install ros-melodic-navigation
git clone https://github.com/ros-perception/openslam_gmapping ~/catkin_ws/src/openslam_gmapping
git clone https://github.com/ros-perception/slam_gmapping ~/catkin_ws/src/slam_gmapping
```

- ROS-kinetic
```bash
sudo apt-get install ros-kinetic-navigation ros-kinetic-gmapping
```

### teb_local_planner
https://github.com/MLCS-Yonsei/teb_local_planner

TEB local planner w/ kinematic constraints for Mecanum-wheeled mobile robot


<br/>

## Run

### V-rep simulator
```bash
roslaunch vrep_holonomic_bringup bringup.launch vrep_path:=/path/to/vrep
```

### SLAM (gmapping)
```bash
roslaunch vrep_holonomic_slam gmapping.launch
```

### Localization only (AMCL)
```bash
roslaunch vrep_holonomic_slam amcl.launch
```

### MPC
multiple shooting:
```bash
roslaunch vrep_holonomic_mpc mpc_ms.launch casadi_path:=/path/to/casadi
```
single shooting:
```bash
roslaunch vrep_holonomic_mpc mpc_ss.launch casadi_path:=/path/to/casadi
```


<br/>

## Test environments (TTAK.KO-10.0809)

### V-rep
- Environment 1
```bash
roslaunch vrep_holonomic_bringup bringup.launch vrep_path:=/path/to/vrep vrep_scene_file:=~/catkin_ws/src/vrep/holonomic/vrep_holonomic_bringup/scenes/test_1.ttt
```
- Environment 2
```bash
roslaunch vrep_holonomic_bringup bringup.launch vrep_path:=/path/to/vrep vrep_scene_file:=~/catkin_ws/src/vrep/holonomic/vrep_holonomic_bringup/scenes/test_2.ttt
```
- Environment 3
```bash
roslaunch vrep_holonomic_bringup bringup.launch vrep_path:=/path/to/vrep vrep_scene_file:=~/catkin_ws/src/vrep/holonomic/vrep_holonomic_bringup/scenes/test_3.ttt
```

### AMCL
```bash
roslaunch vrep_holonomic_slam amcl.launch map_file:=~/catkin_ws/src/vrep/holonomic/vrep_holonomic_slam/maps/test.yaml
```
