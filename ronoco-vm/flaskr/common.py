"""
This file implements the common endpoint
"""
import time
from threading import Timer

from flask import Blueprint
from flask import request

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
        self.rt = RepeatedTimer(30.0, self.send_states)  # threading.Timer
        self.robot = False
        self.rviz = False

    def send_states(self):
        """
        This function checks if the state of the robot or rviz has changed. If so, it sends a message to the
        websocket's states channel
        """
        if self.robot != self.robot_state() or self.rviz != self.rviz_state():
            self.robot = self.robot_state()
            self.rviz = self.rviz_state()
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
        self.shutdown_server()
        self.rt.stop()
        return {"Info": 'Server shutting down...'}, 200

    @staticmethod
    def shutdown_server():
        """
        Call werkzeug.server.shutdown to shutdown server
        """
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()


class RepeatedTimer(object):
    """
    This class allows to create a repeated thread thanks to a Timer
    """

    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        """
        Auto start
        """
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        """
        Starter
        """
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        """
        Stopper
        """
        self._timer.cancel()
        self.is_running = False
