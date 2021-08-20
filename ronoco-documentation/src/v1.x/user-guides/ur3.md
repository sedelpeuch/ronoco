# Universal robot 3

## Configuring ROS with universal robots

This driver requires a system setup with ROS. It is recommended to use Ubuntu 18.04 with ROS melodic however using
Ubuntu 20.04 with ROS noectic work also. To install ROS noetic on Ubuntu 20.04 see [the relative page](installation.md)

First it is necessary to download and install
[Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) by performing the commands
below

```bash
# source global ros
source /opt/ros/<ros_distro>/setup.<bash/zsh>

# create a catkin workspace if necessary
mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone fork of the description. This is currently necessary, until the changes are merged upstream.
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

# install dependencies
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace
catkin_make

# activate the workspace (ie: source it)
source devel/setup.<bash/zsh>
```

### Prepare the robot

For using the `ur_robot_driver` with a real robot you need to install the **externalcontrol-1.0.4.urcap** see
[Installing
a URCap on a CB3 robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md)
or
[Installing a URCap on a e-Serries robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md).

### Prepare the ROS PC

Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script :

```bash
roslaunch ur_calibration calibration_correction.launch robot_ip:=<robot_ip> \
target_filename:="${HOME}/calibration.yaml"
```

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot.

## Quick Start

Once the driver is built and the **externalcontrol** URCap is installed on the robot, you are good to go ahead starting the driver. To actually start the robot driver use one of the existing launch files

```bash
roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \ 
kinematics_config:="${HOME}/calibration.yaml"
```

Where **<robot_type>** is one of *ur3*, *ur5*, *ur10*, *ur3e*, *ur5e*, *ur10e*, *ur16e*.

If everything went well you can run `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller` to check that the communication between the PC and the robot is effective.

Once this is done you can start **MoveIt**, to do this run the command `roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch`. Once **MoveIt** is running it is possible to start **rviz** via the command `roslaunch ur3_moveit_config moveit_rviz.launch config:=true`.

There have been problems with controllers running the path from **rviz**, for more information see the [logs](log.log)

## Using ronoco with a universal robot

Once the installation procedure of ROS on a universal robots model is completed, you just have to start ronoco by following the [Quick Start](quick-start.md) with the followings arguments :
- *commander*: manpiulator
- *compliant_mode*: manual

Once started you can use ronoco in the usual way.

### Limitations

Universal robots do not support the use of "free mode" at the same time as the execution of a program, i.e. when executing the **ExternalControl** block it is not possible to use the "free mode" function. Thus the *record* and *replay* blocks cannot be used on a universal robot.

WIP : create record & replay in rviz