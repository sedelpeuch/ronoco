# Getting started

To use ronoco it is necessary to launch the ROS modules corresponding to your robot and then the different modules of Ronoco.

Regarding the launching of ROS modules the procedure is different for each robot, to make Ronoco work it is necessary to have a roscore and Moveit running. See the different examples on [Poppy Ergo Jr](poppy.md) or [Universal Robots 3](ur3.md) for more details.

It is then necessary to launch the three modules.
```bash
# Launching of nodered
node-red
# Launching of ronoco-vm
cd $HOME/catkin_ws/src/Ronoco/ronoco-vm/
python3 ronoco_vm/run.py
# Launching of ronoco-ui
cd $HOME/catkin_ws/src/Ronoco/ronoco-ui/
npm start
```

Once all the modules are running go to your [localhost:8080](http://localhost:8080/) and you will arrive on the Ronoco page:

![ronoco](ronoco.png)

Work in progress: launching Ronoco using a roslaunch
