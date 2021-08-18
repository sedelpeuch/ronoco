"""
This file gathers all the configuration variables until ronoco-config is created
"""

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction
from moveit_commander import MoveGroupCommander

finished = None
socketio = None

# Level of debug
debug = 4

ronoco_mode = rospy.get_param("mode")

if ronoco_mode == "manipulator":
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

elif ronoco_mode == "rolling":
    namespace = rospy.get_param("namespace")
    move_base = namespace + "/move_base"
    cmd_vel = namespace + "/cmd_vel"
    amcl_pose = namespace + "/amcl_pose"
    commander = actionlib.SimpleActionClient(move_base, MoveBaseAction)
