"""
Definition of all callback treatement for ros topics
"""
import config

position_simulation = {}
position_amcl = {}


def position_callback(data):
    """
    Define treatment when a message is post on /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
    topic for manipulator robot or on /clicked_point for rolling robot
    :param data: the message (InteractiveMarkerUpdate for manipulator PointStamped for rolling)
    """
    global position_simulation
    pose = {}
    orientation = {}
    if config.ronoco_mode == "manipulator":
        try:
            if config.ronoco_mode == "manipulator":
                pose = data.poses[0].pose.position
                orientation = data.poses[0].pose.orientation

            position_simulation = {"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                                   "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z,
                                                   "w": orientation.w}}
        except IndexError:
            pass

    elif config.ronoco_mode == "rolling":
        try:
            pose = data.point
            position_simulation = {"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                                   "orientation": {"x": 0.0, "y": 0.0, "z": 0.05, "w": 0.05}}
        except IndexError:
            pass


def amcl_callback(data):
    """
    Define treatment when a message is post on /amcl_pose
    topic for manipulator robot or on /clicked_point for rolling robot
    :param data: the message (InteractiveMarkerUpdate for manipulator PointStamped for rolling)
    """
    global position_amcl
    try:
        pose = data.pose.pose.position
        orientation = data.pose.pose.orientation
        position_amcl = {"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                         "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z,
                                         "w": orientation.w}}
    except IndexError:
        pass
