# <center> <img src="logo_full_white.png" width="60%"/> </center>

<video controls autoplay loop muted playsinline>
  <source src="ronoco.mp4" type="video/mp4">
  <p>Votre navigateur ne prend pas en charge les vidéos HTML5.
     Voici <a href="myVideo.mp4">un lien pour télécharger la vidéo</a>.</p>
</video>

With **ROS**, creating a robotic system means developing programs in **C++** or **Python**, making ROS **inaccessible** to all non-developer experts: process engineers, industrialization engineers...

Writing code is indeed becoming essential for complex robots.

However, for several years now, an alternative method has been making headway: **no-code**. This consists of designing computer programs without writing code, often via a graphic interface. This project consists of developing a **prototype of a "no-code ROS"** that can be used to program **manipulative** robots under **MoveIt** and **rolling robots**.

Ronoco connects to the ROS ecosystem and generates standard ROS messages, maintaining the interoperable ROS (**robot-agnostic**) DNA.

The ronoco project is divide into three distinct modules, each fulfilling different and complementary roles to provide a graphical interface for designing ROS manipulation programs.

The three modules are **ronoco-vm** a ROS package consisting of a flask API that interprets the sequence of blocks in the GUI and translates them into ROS code. The second is **ronoco-nodered**, an extension to the [Node-RED](https://nodered.org/) application that allows you to graphically create action sequences using behaviour trees

The last one is **ronoco-ui** a web client allowing to use the two previous modules. It provides utilities for the robot (registering positions, launching a program, stopping the server etc). Moreover, it allows defining programs using a no-code interface like NodeRed or Scratch (WIP)

## Can ronoco be used on my robot?

To use ronoco with your robot it is necessary that it has the following specifications:
1. For a manipulator robot it is necessary that it is compatible with **MoveIt**. Ronoco mainly uses MoveIt to control the manipulator robots and the rviz visualization topics
2. For a rolling robot it is necessary that it is compatible with **MoveBase**. Ronoco uses the SimpleActionClient of move base for navigation. The robot will also have to provide a topic where it publishes its current position (typically /amcl_pose) as well as a topic allowing it to be teleoperated (typically /cmd_vel)

If your robot meets these specifications you can use ronoco with it!