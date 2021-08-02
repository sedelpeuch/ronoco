# Add your own robot

To add your own robot, simply modify the configuration file. The **config.py** file can be found in the `$HOME/catkin_ws/src/ronoco/ronoco-vm/ronoco_vm/` folder

```py
socketio = None

# Level of debug
debug = 4

# Move group for moveit
move_group = "arm_and_finger" # Poppy Ergo Jr 
move_group = "manipulator" # UR3
move_group = "maniupaltor" # KR6

# Compliance mode
mode = "/set_compliant" # Poppy Ergo Jr
mode = "manual" # UR3
mode = None # KR6
```

This configuration file will disappear in the next version of Ronoco in favour of parameters in a roslaunch