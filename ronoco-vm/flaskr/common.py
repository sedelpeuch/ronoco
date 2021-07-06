"""
This file implements the common endpoint
"""
from flask import Blueprint
from flask import request
from werkzeug.exceptions import NotFound

import rospy
from roscpp.srv import GetLoggers


class Common:
    """
    Definition of common endpoint
    """
    def __init__(self):
        self.bp = Blueprint('common_endpoint', __name__, url_prefix='/')

        self.bp.route('/', methods=['GET'])(self.index)
        self.bp.route('/robot_state', methods=['GET'])(self.robot_state)
        self.bp.route('/shutdown')(self.shutdown)

    @staticmethod
    def index():
        """
        GET Method

        ROUTE /

        TODO : change index behaviour
        :return: if everything is ok : 'Hello World !' else an HttpError
        """
        return {"Success": "Server is running"}, 200

    @staticmethod
    def robot_state():
        """
        ROUTE /robot_state

        Check if you can communicate with a ros master
            + Use rosservice /rosout/get_loggers
            + Node: /rosout
            + Type: roscpp/GetLoggers
            + Args:

        :return: {'robot_state': True} if communication with rosmaster is possible, NotFound exception else
        """
        get_loggers = rospy.ServiceProxy('rosout/get_loggers', GetLoggers)
        try:
            get_loggers()
        except rospy.service.ServiceException:
            raise NotFound()
        return {'robot_state': True}, 200

    def shutdown(self):
        """
        Shutdown server
        """
        self.shutdown_server()
        return 'Server shutting down...'

    @staticmethod
    def shutdown_server():
        """
        Call werkzeug.server.shutdown to shutdown server
        """
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('Not running with the Werkzeug Server')
        func()
