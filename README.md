<img src="logo.svg" width="5%"/> Ronoco [![CI](https://github.com/Sdelpeuch/ronoco/actions/workflows/main.yml/badge.svg?branch=gh-pages)](https://github.com/Sdelpeuch/ronoco/actions/workflows/main.yml)
=========================================================================================================================================================================================================

With *ROS*, creating a robotic system means developing programs in *C++* or *Python*, making ROS *inaccessible* to all non-developer experts: process engineers, industrialization engineers...

Writing code is indeed becoming essential for complex robots.

However, for several years now, an alternative method has been making headway: **no-code**. This consists of designing computer programs without writing code, often via a graphic interface. This project consists of developing a *prototype of a "no-code ROS"* that can be used to program *manipulative* robots under *MoveIt* and *rolling robots*.

Ronoco connects to the ROS ecosystem and generates standard ROS messages, maintaining the interoperable ROS (*robot-agnostic*) DNA.

The ronoco project is divide into three distinct modules, each fulfilling different and complementary roles to provide a graphical interface for designing ROS manipulation programs.

The three modules are **ronoco-vm** a ROS package consisting of a flask API that interprets the sequence of blocks in the GUI and translates them into ROS code. The second is **ronoco-nodered** a

The last one is **ronoco-ui** a web client allowing to use the two previous modules. It provides utilities for the robot (registering positions, launching a program, stopping the server etc). On the other hand it allows defining programs using a no-code interface like NodeRed or Scratch (WIP)

## Can ronoco be used on my robot?

To use ronoco with your robot it is necessary that it has the following specifications:
1. For a manipulator robot it is necessary that it is compatible with **MoveIt**. Ronoco mainly uses MoveIt to control the manipulator robots and the rviz visualization topics
2. For a rolling robot it is necessary that it is compatible with **MoveBase**. Ronoco uses the SimpleActionClient of move base for navigation. The robot will also have to provide a topic where it publishes its current position (typically /amcl_pose) as well as a topic allowing it to be teleoperated (typically /cmd_vel)

If your robot meets these specifications you can use ronoco with it!

## Documentation

The project documentation is available at  [https://sdelpeuch.github.io/Ronoco/](https://sdelpeuch.github.io/ronoco/). It includes guides for users (installation, start-up, use, examples on robots), guides for developers as well as a precise documentation of the different modules.

## Installation

To install ronoco you need the following packages:

- npm
- python3
- ROS noetic (also works on ROS melodic)

To start it is necessary to clone the project in the catkin workspace
```bash
cd $HOME/catkin_ws/src/
git clone https://github.com/Sdelpeuch/ronoco.git
```

Then you have to install the three modules that constitute ronoco. Firstly *ronoco-nodered* allowing the user to define the behaviour trees of the robot (see How to use it and How to create a behaviour tree). This module depends on Node-RED, so it is necessary to install the framework and then the blocks specific to ronoco.

```bash
sudo npm install -g --unsafe-perm node-red
mkdir $HOME/.node-red/
cd $HOME/.node-red/
npm install $HOME/catkin_ws/src/ronoco/ronoco-nodered/
```

Once *ronoco-nodered* is installed it is necessary to install the web client inside *ronoco-ui*

```bash
cd $HOME/catkin_ws/src/ronoco/ronoco-ui/
npm install
```

Finally, all that remains is to install the application engine found in *ronoco-vm*.

```bash
cd $HOME/catkin_ws/src/ronoco/ronoco-vm/
pip3 install -r requirements.txt
```

It is necessary to download a socketio client in the static folder of the API

```bash
cd $HOME/catkin_ws/src/ronoco/ronoco-vm/ronoco_vm/static/
mkdir socket.io && cd socket.io
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js.map
```

Before using ronoco it is necessary to compile the ROS workspace

```bash
cd $HOME/catkin_ws/
catkin_make
source devel/setup.bash
```

### Set up Rviz (optionnal)

If you want to view the different points and paths defined by ronoco in rviz it is necessary to add two markers in rviz.

To add a marker in rviz : click on "add" at the bottom left of the screen. A menu opens. Click on "Marker". Then in the menu on the left of your screen (Displays) find "Marker". Scroll down the menu and set the "Marker topic" field to :

1. to view the recorded points: "visualization_marker".
2. (rolling robots only) to view the paths travelled: "path_coverage_marker"

<center>
<img src="ronoco-documentation/src/v2.x/static/marker.gif"></img>
</center>

## Quick Start

### Manipulator mode

To launch the project with a manipulator arm, simply run the following command:

```bash
roslaunch ronoco manipulator.launch commander:=string compliant_mode:=string end_effector:=string
```

With as argument :
- *commander*: the name of the move_group in MoveIt
- *compliant_mode*: *manual* if the robot can go into compliant mode manually, *None* if the robot cannot go into compliant mode, or *the name of the service* to put it in and out of compliant mode.
- *end_effector*: the name of the service to manipulate the effector, e.g. "wsg_50_driver/move". (optional)

### Rolling mode

To launch the project with a rolling robot, simply run the following command:

```bash
roslaunch ronoco rolling.launch namespace:=string
```

With as argument :
- *namespace*: the namespace for your robot without last / (default " ")

Once all the modules are running go to your [localhost:8080](http://localhost:8080/) and you will arrive at the ronoco page:

![ronoco](ronoco-documentation/src/static/ronoco.png)
