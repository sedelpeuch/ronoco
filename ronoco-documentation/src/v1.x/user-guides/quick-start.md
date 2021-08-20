# Getting started

## Quick-start : roslaunch

To launch the project, simply run the following command:

```bash
roslaunch ronoco ronoco.launch commander:=string compliant_mode:=string end_effector:=string
```

With as argument :
- *commander*: the name of the move_group in MoveIt
- *compliant_mode*: *manual* if the robot can go into compliant mode manually, *None* if the robot cannot go into compliant mode, or *the name of the service* to put it in and out of compliant mode.
- *end_effector*: the name of the service to manipulate the effector, e.g. "wsg_50_driver/move". It is not necessary to fill in this field

## Manual start

To use ronoco it is necessary to launch the ROS modules corresponding to your robot and then the different modules of ronoco.

Regarding the launching of ROS modules the procedure is different for each robot, to make ronoco work it is necessary to have a roscore and Moveit running. See the different examples on [Poppy Ergo Jr](poppy.md) or [Universal Robots 3](ur3.md) for more details.

First set parameters in rosparam
```bash
rosparam set commander string
rosparam set compliant_mode string
rosparam set end_effector string
```

It is then necessary to launch the three modules.
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
