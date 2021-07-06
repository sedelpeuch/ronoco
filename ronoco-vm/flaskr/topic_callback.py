"""
Definition of all callback treatement for ros topics
"""
position = {}


def position_callback(data):
    """
    Define treatment when a message is post on /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
    topic
    :param data: the message
    """
    global position
    position = {}
    try:
        pose = data.poses[0].pose.position
        orientation = data.poses[0].pose.orientation
        position = {"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                    "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}}
    except IndexError:
        pass
