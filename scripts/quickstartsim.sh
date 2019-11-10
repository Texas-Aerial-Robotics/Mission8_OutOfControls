#!/bin/bash -x

package='obs_avoid'
launchFile='droneOnly.launch'
if [[ $# == 2 ]]; then
	package=$1
	launchFile=$2
fi
roslaunch mission8_sim $launchFile &

sleep 5

mkfifo /tmp/status
exec 3<>/tmp/status
trap "rm -f /tmp/status" EXIT
function setGuided
{	
	#while [[ read var ||  '$var' != 'Flight battery 100 percent' ]]; do
		#statements
		

	#done
	grep -m 2 'using GPS' <&3
#	grep -q -m 2 'using GPS' </tmp/status
	
#	sleep 1m # do this if it stilll does not work
	
	roslaunch out_of_controls apm.launch > /dev/null 2>&1 &

	rosrun out_of_controls $package > /dev/tty &
	#for tracking you do rostopic pub /Points geometry_msgs/Point '{x: -1.0, y: 0.0, z: 0.0}'

	sleep 5

	echo 'mode guided'

#	grep -q 'Flight battery 90 percent' </tmp/status

	wait


	
}
cd ~/ardupilot/ArduCopter/ && (setGuided | sim_vehicle.py -f gazebo-iris -I0) | tee /dev/tty >&3  &
#cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -f gazebo-iris -I0 </tmp/status | tee >(setGuided >/tmp/status)  &

wait