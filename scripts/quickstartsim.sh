#!/bin/bash

roslaunch mission8_sim droneOnly.launch &

sleep 5

cd ~/ardupilot/ArduCopter/ && echo 'mode GUIDED' | sim_vehicle.py -f gazebo-iris --console -I0  &

roslaunch out_of_controls apm.launch &

rosrun out_of_controls tracking

wait