<img src="logo.svg" width="5%"/> Ronoco [![CI](https://github.com/Sdelpeuch/Ronoco/actions/workflows/main.yml/badge.svg?branch=gh-pages)](https://github.com/Sdelpeuch/Ronoco/actions/workflows/main.yml)
========================

With *ROS*, creating a robotic system means developing programs in *C++* or *Python*, making ROS *inaccessible* to all non-developer experts: process engineers, industrialization engineers...

Writing code is indeed becoming essential for complex robots.

However, for several years now, an alternative method has been making headway: **no-code**. This consists of designing computer programs without writing code, often via a graphic interface. This project consists of developing a *prototype of a "no-code ROS"* that can be used to program *manipulative* robots under *MoveIt* and *rolling robots*.

Ronoco connects to the ROS ecosystem and generates standard ROS messages, maintaining the interoperable ROS (*robot-agnostic*) DNA.

The ronoco project is divide into three distinct modules, each fulfilling different and complementary roles to provide a graphical interface for designing ROS manipulation programs.

The three modules are **ronoco-vm** a ROS package consisting of a flask API that interprets the sequence of blocks in the GUI and translates them into ROS code. The second is **ronoco-nodered** a

The last one is **ronoco-ui** a web client allowing to use the two previous modules. It provides utilities for the robot (registering positions, launching a program, stopping the server etc). On the other hand it allows defining programs using a no-code interface like NodeRed or Scratch (WIP)

## Documentation

The project documentation is available at  [https://sdelpeuch.github.io/Ronoco/](https://sdelpeuch.github.io/Ronoco/). It includes guides for users (installation, start-up, use, examples on robots), guides for developers as well as a precise documentation of the different modules.

## Installation

To install Ronoco you need the following packages:

- npm
- python3
- ROS noetic (also works on ROS melodic)

To start it is necessary to clone the project in the catkin workspace
```bash
cd $HOME/catkin_ws/src/
git clone https://github.com/Sdelpeuch/Ronoco.git
```

Then you have to install the three modules that constitute Ronoco. Firstly *Ronoco-nodered* allowing the user to define the behaviour trees of the robot (see How to use it and How to create a behaviour tree). This module depends on Node-RED, so it is necessary to install the framework and then the blocks specific to Ronoco.

```bash
sudo npm install -g --unsafe-perm node-red
mkdir $HOME/.node-red/
cd $HOME/.node-red/
npm install $HOME/catkin_ws/src/Ronoco/ronoco-nodered/
```

Once *Ronoco-nodered* is installed it is necessary to install the web client inside *Ronoco-ui*

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-ui/
npm install
```

Finally, all that remains is to install the application engine found in *Ronoco-vm*.

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-vm/
pip3 install -r requirements.txt
```

It is necessary to download a socketio client in the static folder of the API

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-vm/ronoco_vm/static/
mkdir socket.io
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js.map
```

Before using Ronoco it is necessary to compile the ROS workspace

```bash
cd $HOME/catkin_ws/
catkin_make
source devel/setup.<bash/zsh>
```

## Quick Start