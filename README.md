# vrep_holonomic

ROS V-rep simulation package for Mechanum-wheeled mobile robot



## Requirements

### ROS
http://wiki.ros.org/ROS/Installation

### v-rep
http://www.coppeliarobotics.com/downloads.html

### CasADi
This package uses the python 2.7  version of Casadi
https://web.casadi.org/get/



## Run

### V-rep simulator
```bash
roslaunch vrep_holonomic_bringup bringup.launch vrep_path:=/path/to/vrep
```

### SLAM (gmapping)
```bash
roslaunch vrep_holonomic_slam gmapping.launch
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
