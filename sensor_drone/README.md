# Sensor Drone
*More info can be found on the [wiki](https://github.com/ParagonAutonomus/ProjectParagon/wiki/Sensor-Drone).*

### Installation Guide
*[Ubuntu 22.04.5 LTS](https://mirror.pilotfiber.com/ubuntu-iso/22.04.5/) is recommended*
1. Install prerequisite libraries
- [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Ardupilot Installation Guide](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)
- [Gazebo Harmonic Installation Guide](https://gazebosim.org/docs/harmonic/install/)
- [Ardupilot Gazebo Plugin Installation Guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)

2. Clone repository
``` bash
git clone https://github.com/ParagonAutonomus/ProjectParagon.git
```

3. Install [mavros](https://github.com/mavlink/mavros/tree/ros2/mavros) ~~and [ros_gz](https://github.com/gazebosim/ros_gz)~~ from source
``` bash
sudo apt install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon

cd ~/ProjectParagon/sensor_drone/drone_ws/

# Install MAVLink
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos

# Install MAVROS
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos

# Create workspace & deps
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos
rosdep install --from-paths src --ignore-src -y

# Install GeographicLib datasets
sudo ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Build source
colcon build

# Add source to ~/.bash and copy over simulation model
echo "source ~/ProjectParagon/sensor_drone/drone_ws/install/setup.bash" >> ~/.bashrc
cp ~/ProjectParagon/sensor_drone/model.sdf ~/gz_ws/src/ardupilot_gazebo/models/iris_with_standoffs/
```

4. Launch simulation
``` bash
gz sim -v4 -r iris_runway.sdf
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
ros2 launch mavros apm.launch fcu_url:=udp://:14550@
# or
cd ~/ProjectParagon/sensor_drone/
./launch_sim.sh
```

Launch `auto_uav` package:
``` bash
ros2 launch auto_uav auto_uav_launch.py
```

### Development Guide
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)

Sample `.vscode/c_cpp_properties.json` configuration:
``` json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/humble/include/**",
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
