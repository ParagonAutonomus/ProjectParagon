# Sensor Drone
*More info can be found on the [wiki](https://github.com/ParagonAutonomus/ProjectParagon/wiki/Sensor-Drone).*

### Installation Guide
*[Ubuntu 22.04.5 LTS](https://mirror.pilotfiber.com/ubuntu-iso/22.04.5/) is recommended*
1. Install prerequisite libraries
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Ardupilot Setup Guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/harmonic/install/)
- [Ardupilot Gazebo Plugin Installation Guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

Verify installation by running SITL with Gazebo.

Run Gazebo:
``` bash
gz sim -v4 -r iris_runway.sdf
```
Run SITL:
``` bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```
Test SITL Commands:
``` bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```
2. Clone ProjectParagon repository
``` bash
git clone https://github.com/ParagonAutonomus/ProjectParagon.git
```
3. TODO ...

### Simulation Guide

### Development Guide