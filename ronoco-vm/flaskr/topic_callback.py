position = {}


def position_callback(data):
    global position
    position = {}
    try:
        pose = data.poses[0].pose.position
        orientation = data.poses[0].pose.orientation
        position = {"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                    "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}}
    except IndexError:
        pass
