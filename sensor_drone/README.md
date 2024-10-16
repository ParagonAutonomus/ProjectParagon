# Sensor Drone
*More info can be found on the [wiki](https://github.com/ParagonAutonomus/ProjectParagon/wiki/Sensor-Drone).*

### TODOS
- [ ] Gazebo sensor interface
- [ ] MAVLink interface with Mission Planner
- [ ] Mission context and controller
- [ ] Takeoff, landing, and return procedures
- [ ] Battery and range calculations
- [ ] Path finding implementation
- [ ] Wildfire detection from thermal imager
- [ ] Camera feed generation
- [ ] Wildfire classification model
- [ ] Target waypoint generation from risk-map
- [ ] Custom Mission Planner and monitoring system???

### Installation Guide
*[Ubuntu 22.04.5 LTS](https://mirror.pilotfiber.com/ubuntu-iso/22.04.5/) is recommended*
1. Install prerequisite libraries
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Ardupilot Installation Guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [Ardupilot ROS 2 Workspace Installation Guide](https://ardupilot.org/dev/docs/ros2.html)
- [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/harmonic/install/)
- [Ardupilot Gazebo Plugin Installation Guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

*Verify your installation, the following should run successfully.*

Source these ROS workspaces in `~/.bashrc`
```
source /opt/ros/humble/setup.bash     # core ros workspace
source ~/ardu_ws/install/setup.bash   # ardupilot dds workspace
```

Run Gazebo:
``` bash
gz sim -v4 -r iris_runway.sdf
```
Run microROS agent (UDP for SITL):
```
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

2. Clone Repository
``` bash
git clone https://github.com/ParagonAutonomus/ProjectParagon.git
```
3. Build and source workspace
VSCode example `c_cpp_properties.json` cpp workspace configuration file
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**", 
                "~/ardu_ws/install/ardupilot_msgs/include/**",
                "/usr/include/**"        
            ],
            "defines": [],
            "compilerPath": "/usr/lib/ccache/clang-14",
            "cStandard": "c17",
            "cppStandard": "c++14",
            "intelliSenseMode": "linux-clang-x64"
        }
    ],
    "version": 4
}
```

Build and source `drone_ws` workspace
``` bash
cd ~/ProjectParagon/sensor_drone/drone_ws/
colcon build
echo "source ~/ProjectParagon/sensor_drone/drone_ws/install/setup.bash" >> ~/.bashrc
```

``` bash
ros2 pkg list
```

3. Simulation setup

Replace default model
``` bash
cp ~/ProjectParagon/sensor_drone/model.sdf ~/gz_ws/src/ardupilot_gazebo/models/iris_with_standoffs/
```

Test SITL Commands
``` bash
module load message
mode guided
arm throttle
takeoff 5
message SET_POSITION_TARGET_LOCAL_NED 0 0 0 7 3576 20 0 0 0 0 0 0 0 0 0 0
```

4. Running `test_controller` node

### Development Guide
- [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)
