"""
This file gathers all the configuration variables until ronoco-config is created
"""
import rospy
from moveit_commander import MoveGroupCommander

socketio = None

# Level of debug
debug = 4

# Move group for moveit
move_group = rospy.get_param("commander")

# Compliance mode
mode = rospy.get_param("compliant_mode")
if mode == "None":
    mode = None

# Gripper
end_effector = None
try:
    end_effector = rospy.get_param("end_effector")
except KeyError:
    pass

try:
    commander = MoveGroupCommander(move_group, wait_for_servers=20)
except RuntimeError:
    commander = None
