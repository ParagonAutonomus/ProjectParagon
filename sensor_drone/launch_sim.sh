#!/bin/bash

gnome-terminal -- bash -c "gz sim -v4 -r iris_runway.sdf; exec bash"
gnome-terminal -- bash -c "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console; exec bash"
gnome-terminal -- bash -c "ros2 launch mavros apm.launch fcu_url:=udp://:14550@; exec bash"

echo "All commands launched in separate terminal windows."