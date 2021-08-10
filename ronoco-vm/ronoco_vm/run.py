"""
This file is the entry point of flask api. Launch python3 ronoco_vm/run.py to run flask server
"""
import os
import time

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
from werkzeug.debug import DebuggedApplication

import config
import rospy
import topic_callback
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import InteractiveMarkerUpdate


class RonocoVm:
    """
    Define and setup flask server and ros topic subscriber / publisher for ronoco-vm
    """

    def __init__(self):
        """
        Launch flask server when RonocoVm is created (this constructor uses SocketIO)
        """
        self.app = None
        self.create_app()
        config.socketio = SocketIO(self.app, logger=False, cors_allowed_origins='*')
        self.subscribe_topic()
        self.setup_app()
        rospy.init_node('user')
        rospy.loginfo("User root is serving the Web app")
        config.socketio.run(self.app)

    def create_app(self, test_config=None):
        """
        Build a Flask instance and configure it
        :param test_config: path to configuration file (Default : None)
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

    def setup_app(self):
        """
        Register blueprint in app.

        The class attribute "app" must contain an Flask instance
        :return: None
        """
        import cartesian_point
        self.app.register_blueprint(cartesian_point.CartesianPoint().bp)

        import control
        self.app.register_blueprint(control.Control().bp)

        import common
        self.app.register_blueprint(common.Common().bp)

        import free
        self.app.register_blueprint(free.bp)

        CORS(self.app)

    @staticmethod
    def subscribe_topic():
        """
        Uses rospy to subscribe to the different topics needed by the API
        :return: None
        """
        if config.ronoco_mode == "manipulator":
            rospy.Subscriber(
                "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
                InteractiveMarkerUpdate, topic_callback.position_callback)
            time.sleep(0.5)
        elif config.ronoco_mode == "rolling":
            rospy.Subscriber("/clicked_point", PointStamped, topic_callback.position_callback)
            rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, topic_callback.amcl_callback)


if __name__ == "__main__":
    RonocoVm()
