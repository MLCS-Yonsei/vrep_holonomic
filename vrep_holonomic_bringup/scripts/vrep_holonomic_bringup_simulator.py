#!/usr/bin/env python

import rospy
import time
import subprocess

subprocess.call(
    "rosparam set use_sim_time true",
    shell=True
)

vrep_path = rospy.get_param("/vrep_holonomic_bringup_simulator/vrep_path")
viz = rospy.get_param("/vrep_holonomic_bringup_simulator/visuallization")
scene = rospy.get_param("/vrep_holonomic_bringup_simulator/scene_file")

if viz:
    vrep_exec = vrep_path+"/vrep.sh "
    t_val = 5.0
else:
    vrep_exec = vrep_path+"/vrep.sh -h "
    t_val = 1.0
subprocess.call(
    vrep_exec+"-s -q "+scene+" &",
    shell=True
)
time.sleep(t_val)      