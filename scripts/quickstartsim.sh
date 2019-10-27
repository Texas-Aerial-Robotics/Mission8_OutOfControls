#!/bin/bash

roslaunch mission8_sim droneOnly.launch &

sleep 5

mkfifo /tmp/status
function setGuided
{
	grep -m 4 GPS </tmp/status
	echo 'mode GUIDED'
}
cd ~/ardupilot/ArduCopter/ && setGuided | sim_vehicle.py -f gazebo-iris -I0 >/tmp/status &

roslaunch out_of_controls apm.launch &

rosrun out_of_controls tracking &

echo 'waiting for GPS connection'

while true; do;
	echo '[q]uit [k]ill'
	in= read
	[[in=='q']] && break
	if [[in=='k']]; then
		pkill -s 0 
		break
	fi
done
