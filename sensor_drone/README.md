# Sensor Drone
*More info can be found on the [wiki](https://github.com/ParagonAutonomus/ProjectParagon/wiki/Sensor-Drone).*

### Installation Guide
*[Ubuntu 22.04.5 LTS](https://mirror.pilotfiber.com/ubuntu-iso/22.04.5/) is recommended*
1. Install prerequisite libraries
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Ardupilot Installation Guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [AP_DDS Installation Guide](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_DDS)
- [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/harmonic/install/)
- [Ardupilot Gazebo Plugin Installation Guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

*Verify your installation, the following should run successfully.*

Run Gazebo:
``` bash
gz sim -v4 -r iris_runway.sdf
```
Run microROS agent (UDP for SITL):
```
cd ~/ardupilot/libraries/AP_DDS
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```
Run SITL with DDS enabled:
``` bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --enable-dds
```
Verify `/ardupilot_dds` node is running:
``` bash
ros2 node list
```

2. Clone ProjectParagon repository
``` bash
git clone https://github.com/ParagonAutonomus/ProjectParagon.git
```
3. TODO ...

### Simulation Guide

### Development Guide