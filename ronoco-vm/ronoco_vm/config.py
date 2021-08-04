"""
This file gathers all the configuration variables until ronoco-config is created
"""
import rospy

socketio = None

# Level of debug
debug = 4

# Move group for moveit
move_group = rospy.get_param("commander")

# Compliance mode
mode = rospy.get_param("compliant_mode")
if mode == "None" or "none":
    mode = None
