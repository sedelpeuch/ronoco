"""
This file is the entry point of flask api. Launch python3 flaskr/run.py to run flask server

WIP : shutdown app when KeyboardInterrupt
"""
import os
import time

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
from werkzeug.debug import DebuggedApplication

import rospy
from flaskr import topic_callback
from visualization_msgs.msg import InteractiveMarkerUpdate


class ronoco_vm:
    """
    Define and setup flask server and ros topic suscriber / publisher for ronoco-vm
    """

    def __init__(self):
        """
        Launch flask server when ronoco_vm is created (this constructor uses SocketIO)
        """

        self.create_app()
        socketio = SocketIO(self.app, logger=True)
        rospy.init_node('user')
        self.subscribe_topic()
        rospy.loginfo("User node is serving the Web app")
        socketio.run(self.app)

    def create_app(self, test_config=None):
        """
        Build a Flask instance and configure it
        :param test_config: path to configuration file (Défault : None)
        :return: a Flask instance
        """
        # create and configure the app
        self.app = Flask(__name__, instance_relative_config=True)
        self.app.config.from_mapping(
            SECRET_KEY='dev',
        )
        self.app.debug = True
        self.app.wsgi_app = DebuggedApplication(self.app.wsgi_app, evalex=True)

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
        """
        Register blueprint in app.

        The class attribute "app" must contain an Flask instance
        :return: None
        """
        from flaskr import cartesianpoint
        self.app.register_blueprint(cartesianpoint.CartesianPoint().bp)

        from flaskr import move
        self.app.register_blueprint(move.Move().bp)

        from flaskr import common
        self.app.register_blueprint(common.Common().bp)

        from flaskr import free
        self.app.register_blueprint(free.bp)

        CORS(self.app)

    def subscribe_topic(self):
        """
        Uses rospy to subscribe to the different topics needed by the API
        :return: None
        """
        rospy.Subscriber(
            "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
            InteractiveMarkerUpdate, topic_callback.position_callback)
        time.sleep(0.5)


if __name__ == "__main__":
    ronoco_vm()
