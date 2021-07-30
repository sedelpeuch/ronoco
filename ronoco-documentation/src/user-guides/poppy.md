# Example on Poppy Ergo Jr

This page presents a complete use of ronoco on a [poppy ergo jr](https://github.com/poppy-project/poppy-ergo-jr) it uses the resources of [learn.e.ros4.pro](https://learn.e.ros4.pro/fr/manipulation/ergo-jr/) concerning the Poppy Ergo Jr.

It is necessary to have a ROS installation, a ROS workspace and ronoco to follow this page. Refer to the [installation guide](installation.md) if necessary.

## Prepare Poppy Ergo Jr

Start by [assembling your robot](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/), following the steps for ROS if applicable, then [configure your motors](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/motor-configuration.html#32ter-configurer-les-moteurs-un-par-un-si-vous-utilisez-une-image-ros)

Download the [ROS image](https://github.com/poppy-project/poppy_controllers/releases/download/v1.0/poppy-ergo-jr-ros-melodic.img.zip) to flash an SD card. To flash the image to an SD card:
+ Extract the compressed .zip or .7z file to a folder on the computer
+ Use the software [etcher](https://www.balena.io/etcher/) (or equivalent)
+ In Etcher, "Flash from file", select the image file and the destination (the SD card) and validate

To connect the robot in Wifi :
+ Insert the SD card of the robot in question in your workstation and open the partition named *boot* + Download the file [wpa_supplicant.conf]() in boot and modify it to fill in the right wifi password inside.
+ Download the file [wpa_supplicant.conf](https://files.ros4.pro/wpa_supplicant.conf) in boot and modify it to fill in the right wifi password inside
+ Create an empty file named *.ssh* in the same place in the *boot*.
+ Type the command *sync* and cleanly assign the SD card

These 2 files *wpa_supplicant.conf* and *.ssh* will be deleted the next time the robot is started, indicating that the Wifi connection request has been accepted. It is therefore normal that you will not find them when you look at the boot content again after the first start of the robot.

In case of problems, it is possible to connect an HDMI display to the Raspberry Pi, the network manager is on the top right.

The Wi-Fi connection also works with Android and iOS mobile access points.

Make sure that the `ROS_MASTER_URI` on your workstation and on your poppy point to the same address. If not, modify and source your `.bashrc`.

The controller is already on the robot. You can connect directly to the robot and start it. If you have chosen to point the `ROS_MASTER_URI` at your workstation, do not forget to run a `roscore` before

```bash
ssh pi@poppy.local
roslaunch poppy_controllers control.launch
```
## Start MoveIt with a real robot

Install MoveIt then clone the ROS package **Poppy Ergo Jr MoveIt Configuration**, it contains the code needed to make this robot work with MoveIt :
```bash
sudo apt install ros-noetic-moveit
cd $HOME/catkin_ws/src
git clone https://github.com/poppy-project/poppy_ergo_jr_moveit_config.git
cd $HOME/catkin_ws/
catkin_make 
source $HOME/.bashrc
```

Start MoveIt with `roslaunch` with the `fake_execution` parameter set to `false` to connect to the real robot:
```bash
roslaunch poppy_ergo_jr_moveit_config demo.launch fake_execution:=false gripper:=true
```

Rviz should start with a Poppy Ergo Jr in view corresponding to the state of your robot in real time.

⚠️ At this stage, make sure that the posture of your robot in RViz corresponds to the current posture of the real robot: the angles of the motors and the location of the rivets must correspond in every way to your real robot. It is frequent that robots are incorrectly assembled, in this case close MoveIt and go back to the [assembly guide](https://docs.poppy-project.org/fr/assembly-guides/ergo-jr/) step by step to correct before continuing.

## Start Ronoco

Before starting ronoco it is necessary to provide the robot configuration. Open the file **config.py** in the folder `$HOME/catkin_ws/src/ronoco/ronoco-vm/ronoco_vm/` and copy and paste the following values
```py
"""
This file gathers all the configuration variables until ronoco-config is created
"""

socketio = None

# Level of debug
debug = 4

# Move group for moveit
move_group = "arm_and_finger"

# Compliance mode
mode = "/set_compliant"
```

The two interesting values are `move_group` which can be *arm_and_finger* or *arm* on Poppy Ergo Jr and `mode` which tells ronoco about the compliance capabilities of the robot. In our case the robot can be put into and out of compliance mode via the `/set_compliant` service which we fill in. In the case of a Kuka robot that does not have a compliant mode the value would have been `None`.

WIP: Pass move_group and mode via roslaunch parameters.

To start ronoco you just need to run the following commands in 3 different terminals

```bash
# Launching of nodered in a terminal
node-red
```
```bash
# Launching of ronoco-vm in another terminal
cd $HOME/catkin_ws/src/ronoco/ronoco-vm/
python3 ronoco_vm/run.py
```
```bash
# Launching of ronoco-ui in another terminal
cd $HOME/catkin_ws/src/ronoco/ronoco-ui/
npm start
```

Once all the modules are running go to your [localhost:8080](http://localhost:8080/) and you will arrive at the ronoco page:

![ronoco](../static/ronoco.png)

## Use Ronoco

For a full understanding of this part it is preferable to have followed the tutorials [How to use it](how-to-use-it.md) and [How to create a behaviour tree](bt.md)

The [gif](../static/demo.mp4) below shows a use of Ronoco

![demo](../static/demo.gif)