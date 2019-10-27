#!/bin/bash

roslaunch mission8_sim droneOnly.launch &

sleep 5

mkfifo /tmp/status
function setGuided
{
	grep -m 1 'FCU: EKF2 IMU1 is using GPS' </tmp/status
	
	roslaunch out_of_controls apm.launch &

	rosrun out_of_controls tracking &

	echo 'mode guided'
}
cd ~/ardupilot/ArduCopter/ && setGuided | (sim_vehicle.py -f gazebo-iris -I0) tee >/tmp/status  &



read
pkill -s 0

wait