<img src="logo.svg" width="5%"/> Ronoco
================================================

With **ROS**, creating a robotic system means developing programs in **C++** or **Python**, making ROS **inaccessible** to all non-developer experts: process engineers, industrialization engineers...

Writing code is indeed becoming essential for complex robots.

However, for several years now, an alternative method has been making headway: **no-code**. This consists of designing computer programs without writing code, often via a graphic interface. This project consists of developing a **prototype of a "no-code ROS"** that can be used to program **manipulative** robots under **MoveIt** and **rolling robots**.

Ronoco connects to the ROS ecosystem and generates standard ROS messages, maintaining the interoperable ROS (**robot-agnostic**) DNA.

The ronoco project is divide into three distinct modules, each fulfilling different and complementary roles to provide a graphical interface for designing ROS manipulation programs.

The three modules are **ronoco-vm** a ROS package consisting of a flask API that interprets the sequence of blocks in the GUI and translates them into ROS code. The second is **ronoco-nodered**, an extension to the [Node-RED](https://nodered.org/) application that allows you to graphically create action sequences using behaviour trees

The last one is **ronoco-ui** a web client allowing to use the two previous modules. It provides utilities for the robot (registering positions, launching a program, stopping the server etc). Moreover, it allows defining programs using a no-code interface like NodeRed or Scratch (WIP)

