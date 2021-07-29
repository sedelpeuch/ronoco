# How to use it

This page explains how to use the application to control a robot. It is divided in 3 parts, the first one explains how to use the different buttons of the control interface. The second part explains how to create behaviour trees and the specificities of the different blocks. Finally a complete example of use on a Poppy Ergo Jr is provided.

## Control Interface

<center>
<img src="../static/boutons-ui.png" width="40%" ></img>

Control Interface
</center>


<img src="../static/shutdown.svg" width="5%"></img> Allows to stop the server (WIP: allows to start and stop the whole application)

<img src="../static/circle.svg" width="5%"></img> Represents the status of the server, when it is empty it means that the client has not yet received any information from the server. It is probably still loading.

<img src="../static/circle_red.svg" width="5%"></img> When it is red it means that at least one request to the server has not been answered and therefore the server is not running or is in error

<img src="../static/circle_orange.svg" width="5%"></img> When it is orange it means that ROS is not working as expected. The source of the problem is either the absence of roscore (the rosout/get_loggers service is not available) or the absence of moveit (the move_group/get_loggers service is not available). In this state ronoco is not usable

<img src="../static/circle_yellow.svg" width="5%"></img> When it is yellow it means that Rviz is not communicating the position of the interactive marker. If rviz seems to be working correctly on your machine, simply move the interactive marker to fix the problem. In this state ronoco is usable but the "simulated" button will produce an error

<img src="../static/circle_green.svg" width="5%"></img> When it is green it means that ronoco is fully usable

<img src="../static/play.svg" width="5%"></img> This button asks the server to execute the last behaviour tree deployed from Node-RED

<img src="../static/free.svg" width="5%"></img> Enables the robot to be set to compliant mode. This button can only be used for a robot that has a service to set it to compliant mode (see [add your own robot](own.md) for more details).

<img src="../static/record.svg" width="5%"></img> This group of buttons allows you to record points in the environment. There are two recording modes either via the *simulated* button which records the position of the interactive marker. Or via the *real* button which records the actual position of the robot.

<img src="../static/get.svg" width="5%"></img> This group of buttons allows you to view the different registered positions. It is possible to consult a specific position by filling in the *id* field and then pressing *get* or to consult all the registered positions by using the *get all* button.

<img src="../static/trash.svg" width="5%"></img> This group of buttons allows you to delete the different registered positions. It is possible to delete a specific position by filling in the *id* field and then pressing *delete* or delete all registered positions by using the *clear all* button

## Area for creating behaviour trees

