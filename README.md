# Scan and Polish application
[![Build Status](https://travis-ci.com/rosin-project/automatica18_scan_and_plan_demo.svg?branch=master)](https://travis-ci.com/rosin-project/automatica18_scan_and_plan_demo)
[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

<a href="https://ipa-jfh.github.io/urdf-animation/application_scan_and_plan/">
    <img src="https://user-images.githubusercontent.com/17281534/46005937-aafba700-c0b6-11e8-9d8f-0148392488f1.gif" width="430" height="250">
    >> 3D animation
</a>

## Application implements
- `godel` (Scan and Plan): https://github.com/ros-industrial-consortium/godel
- `ensenso_driver`: https://github.com/ensenso/ros_driver (<a href="http://rosin-project.eu/ftps">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="20" >
</a> [funded](http://rosin-project.eu/ftps))
- `pilz_robots`: https://github.com/PilzDE/pilz_robots
- `pilz_industrial_motion`: https://github.com/PilzDE/pilz_industrial_motion (<a href="http://rosin-project.eu/ftps">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="20" >
</a> [funded](http://rosin-project.eu/ftps))

## Installation

```shell

# Install Ensenso SDK from!
# https://www.ensenso.com/support/sdk-download

# Create a new ROS workspace
mkdir -p ~/snp_demo_ws/src && cd ~/snp_demo_ws/src

# Download demo repository
git clone https://github.com/rosin-project/automatica18_scan_and_plan_demo.git

# Download dependencies
wstool init .
wstool merge ~/snp_demo_ws/src/automatica18_scan_and_plan_demo/snp_prbt.rosinstall
wstool up

# Reset ROS_PACKAGE_PATH
source /opt/ros/kinetic/setup.bash

# Install dependencies 
rosdep update && rosdep install --from-paths ~/snp_demo_ws/src --ignore-src --skip-keys="pilz_modbus pilz_sto_modbus_adapter prbt_pg70_support"


# Build workspace
cd ~/snp_demo_ws && catkin build 

# Source workspace
source ~/snp_demo_ws/devel/setup.bash

```


## Run application

```shell
# Simulation:
roslaunch snp_prbt_bringup application_bringup.launch
# Info: Make sure to press play in Gazebo

# Real robot:
roslaunch snp_prbt_bringup application_bringup.launch sim_robot:=false
# Info: Real robot requires pilz_modbus and pilz_sto_modbus_adapter

```

## Systems settings

### CAN interface
Add in `/etc/network/interfaces` the following config for the [socketcan_interface]( http://wiki.ros.org/socketcan_interface).

```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ip link set $IFACE txqueuelen 15
```


### Remove local scan_parameter cache
Make sure to clear possible local `godel_robot_scan_parameters` since they would overwrite the ones of this repo (`snp_prbt_bringup/config/robot_scan.yaml`).

```shell
rm -f ~/.ros/godel_robot_scan_parameters.msg
```

### Fix QT bug for Robot Planing Panel (might not be required anymore)
```shell
echo "export QT_NO_FT_CACHE=1" >> ~/.bashrc && source ~/.bashrc
```

