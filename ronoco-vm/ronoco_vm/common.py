"""
This file implements the common endpoint
"""
import threading
import time

from flask import Blueprint

import config
import rospy
import topic_callback
from roscpp.srv import GetLoggers


class Common:
    """
    Definition of common endpoint
    """

    def __init__(self):
        self.bp = Blueprint('common_endpoint', __name__, url_prefix='/')

        self.bp.route('/', methods=['GET'])(self.index)
        self.bp.route('/shutdown')(self.shutdown)
        self.rt = threading.Thread(target=self.send_states, daemon=True)
        self.rt.start()

    def send_states(self):
        """
        This function checks if the state of the robot or rviz has changed. If so, it sends a message to the
        websockets states channel
        """
        while True:
            time.sleep(5.0)
            config.socketio.emit('states', {"robot_state": self.robot_state(), "rviz_state": self.rviz_state()},
                                 namespace='/states')

    @staticmethod
    def index():
        """
        GET Method

        ROUTE /

        :return: if everything is ok : 'Server is running, 200' else an HttpError
        """
        return {"Success": "Server is running"}, 200

    @staticmethod
    def robot_state():
        """
        Check if you can communicate with a controller
            + Use rosservice /rosout/get_loggers
            + Node: /rosout
            + Type: roscpp/GetLoggers
            + Args:

        :return: True if communication with rosmaster is possible, False else
        """
        get_loggers = rospy.ServiceProxy('rosout/get_loggers', GetLoggers)
        try:
            get_loggers()
        except rospy.service.ServiceException:
            return False
        return True

    @staticmethod
    def rviz_state():
        """
        Check if rviz send data on topic /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
        /update
        :return: False if rviz doesn't send data, True else
        """
        begin = time.time()
        while topic_callback.position == {}:
            if time.time() - begin > 10:
                return False
        return True

    @staticmethod
    def shutdown():
        """
        GET Method

        ROUTE /shutdown

        Shutdown server with config.socketio.stop() (and shutdown send_states() daemon)
        """
        config.socketio.stop()
        return {"Info": 'Server shutting down...'}, 200
