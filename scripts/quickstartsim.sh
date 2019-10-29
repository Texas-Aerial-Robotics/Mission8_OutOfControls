#!/bin/bash -x

roslaunch mission8_sim droneOnly.launch &

sleep 5

mkfifo /tmp/status
exec 3<>/tmp/status
trap "rm -f /tmp/status" EXIT
function setGuided
{	
	#while [[ read var ||  '$var' != 'Flight battery 100 percent' ]]; do
		#statements
		

	#done
	grep -m 1 'Flight battery 100 percent' <&3
#	grep -q -m 2 'using GPS' </tmp/status
	
#	sleep 1m # do this if it stilll does not work
	
	roslaunch out_of_controls apm.launch > /dev/null 2>&1 &

	rosrun out_of_controls tracking > /dev/tty &

	sleep 5

	echo 'mode guided'

#	grep -q 'Flight battery 90 percent' </tmp/status

	wait


	
}
cd ~/ardupilot/ArduCopter/ && (setGuided | sim_vehicle.py -f gazebo-iris -I0) | tee /dev/tty >&3  &
#cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris -I0 </tmp/status | tee >(setGuided >/tmp/status)  &

wait