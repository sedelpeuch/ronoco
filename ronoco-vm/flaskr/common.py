"""
This file implements the common endpoint
"""
import threading
import time
from threading import Timer

from flask import Blueprint

import rospy
from flaskr import config
from flaskr import topic_callback
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
        websocket's states channel
        """
        while True:
            print("I'm called")
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
        begin = time.time()
        while topic_callback.position == {}:
            if time.time() - begin > 10:
                return False
        return True

    def shutdown(self):
        """
        Shutdown server
        """
        config.socketio.stop()
        return {"Info": 'Server shutting down...'}, 200


class RepeatedTimer:
    def __init__(self, interval, function, *args, **kwargs):
        self.timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self.timer = Timer(self.interval, self._run)
            self.timer.daemon = True
            self.timer.start()
            self.is_running = True

    def stop(self):
        self.timer.cancel()
        self.is_running = False
