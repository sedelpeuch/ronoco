import os
import subprocess
import time

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
from flaskr import topic_callback

import rospy
from visualization_msgs.msg import InteractiveMarkerUpdate


class ronoco_vm:
    app = None
    def __init__(self):
        self.create_app()
        socketio = SocketIO(self.app)
        rospy.init_node('user')
        self.subscribe_topic()
        rospy.loginfo("User node is serving the Web app")
        with self.app.app_context():
            socketio.run(self.app)
        rospy.spin()

    def create_app(self, test_config=None):
        # create and configure the app
        self.app = Flask(__name__, instance_relative_config=True)
        self.app.config.from_mapping(
            SECRET_KEY='dev',
        )

        if test_config is None:
            # load the instance config, if it exists, when not testing
            self.app.config.from_pyfile('config.py', silent=True)
        else:
            # load the test config if passed in
            self.app.config.from_mapping(test_config)

        # ensure the instance folder exists
        try:
            os.makedirs(self.app.instance_path)
        except OSError:
            pass
        self.setup_app()

    def setup_app(self):
        from flaskr import common
        self.app.register_blueprint(common.bp)

        from flaskr import free
        self.app.register_blueprint(free.bp)

        from flaskr import position
        self.app.register_blueprint(position.bp)
        CORS(self.app)

    def subscribe_topic(self):
        response = rospy.Subscriber(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
            InteractiveMarkerUpdate, topic_callback.position_callback)
        time.sleep(0.5)


if __name__ == "__main__":
    ronoco_vm()
