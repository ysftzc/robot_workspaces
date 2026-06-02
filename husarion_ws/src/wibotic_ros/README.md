# wibotic_ros

The repository contains `wibotic_connector_can` and `wibotic_msgs` packages. It reads a CAN Bus thanks to the uavcan library and sends the measurements from [Wibotic Wireless Charger](https://husarion.com/manuals/panther/panther-wch/) to ROS 2.

## Quick start

### Add CAN interface

```bash
sudo slcand -o -s6 -t hw -S 3000000 /dev/ttyACM0
sudo ip link set up can0 type can bitrate 500000
```

### Create workspace

```bash
mkdir ~/husarion_ws
cd ~/husarion_ws
git clone -b ros2 https://github.com/husarion/wibotic_ros.git src/wibotic_ros
```

### Build

```bash
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to wibotic_connector_can --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```
