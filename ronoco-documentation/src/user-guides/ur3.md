# Utiliser ronoco avec un universal robots

## Configurer ROS avec universal robots

This driver requires a system setup with ROS. It is recommended to use Ubuntu 18.04 with ROS melodic however using
Ubuntu 20.04 with ROS noectic work also. To install ROS noetic on Ubuntu 20.04 see [the relative page](installation.md)

Dans un premier temps il est nécessaire de télécharger et installer
[Universal Robots ROS Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) en réalisant les commandes
ci dessous

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

Si tout c'est déroulé correctement vous pouvez lancer `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller` pour vérifier que la communication entre le PC et le robot est effective.

Une fois cela effectué vous pouvez démarrer **MoveIt**, pour cela executez la commande `roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch`. Une fois **MoveIt** en cours d'execution il est possible de démarrer **rviz** via la commande `roslaunch ur3_moveit_config moveit_rviz.launch config:=true`

Il a été constaté des problèmes de controllers avec l'éxecution de trajectoire depuis **rviz**, pour plus d'informations consulter les [logs](log.log)

## Utiliser ronoco avec sur un universal robot

Une fois la procédure d'installation de ROS sur un modèle d'universal robots il suffit de lancer ronoco en suivant le [Quick Start](quick-start.md) une fois lancé vous pouvez utiliser ronoco de manière habituelle.

### Configuration

Pour un universal robot le script de configuration est le suivant

```python
socketio = None

# Level of debug
debug = 4

# Move group for moveit
# move_group = "arm_and_finger"
move_group = "manipulator"

# Compliance mode
# mode = None
# mode = "/set_compliant"
mode = "manual"
```

### Limitations

Les universals robots ne supportent pas l'utilisation du "mode libre" en même temps que l'éxecution d'un programme, autrement dit lorsque l'on éxecute le bloc **ExternalControl** il n'est pas possible d'utiliser la fonction "mode libre". Ainsi les blocs *record* et *replay* ne sont pas utilisables sur un universal robots.