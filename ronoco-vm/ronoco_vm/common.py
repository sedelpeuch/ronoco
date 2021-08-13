"""
This file implements the common endpoint
"""
import threading
import time

from flask import Blueprint

import config
import rospy
import topic_callback
from moveit_commander import MoveGroupCommander
from roscpp.srv import GetLoggers


class Common:
    """
    Definition of common endpoint
    """

    def __init__(self):
        self.bp = Blueprint('common_endpoint', __name__, url_prefix='/')

        self.bp.route('/', methods=['GET'])(self.index)
        self.bp.route('/shutdown')(self.shutdown)
        self.bp.route('/connect', methods=['GET'])(self.connect)
        self.rt = threading.Thread(target=self.send_states, daemon=True)
        self.rt.start()

    def send_states(self):
        """
        Sends every 5 seconds the server status on the websocket "states" channel.

        The server state depends on the operating mode of ronoco.

        In manipulator mode the state of ros, MoveIt, rviz and the MoveGroupCommander

        In rolling mode, the state of ros, rviz and the various topics necessary for the operation of rolling robots
        (cmd_vel, move_base, amcl_pose)
        """
        if config.ronoco_mode == "manipulator":
            while True:
                time.sleep(5.0)
                config.socketio.emit('states', {"ronoco_mode": "manipulator", "ros_state": self.ros_state(),
                                                "moveit_state": self.moveit_state(),
                                                "rviz_state": self.rviz_state(),
                                                "commander_state": self.commander_state()},
                                     namespace='/states')
        elif config.ronoco_mode == "rolling":
            while True:
                time.sleep(5.0)
                config.socketio.emit('states', {"ronoco_mode": "rolling", "ros_state": self.ros_state(),
                                                "rolling_topic": self.rolling_topic(),
                                                "rviz_state": self.rviz_state()},
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
    def ros_state():
        """
        Check if you can communicate with a roscore
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
    def moveit_state():
        """
        Check if you can communicate with moveit
            + Use rosservice /move_group/get_loggers
            + Node: /move_group
            + Type: roscpp/GetLoggers
            + Args:
        :return: True if communication with moveit is possible, False else
        """
        get_loggers = rospy.ServiceProxy('move_group/get_loggers', GetLoggers)
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
        while topic_callback.position_simulation == {}:
            if time.time() - begin > 10:
                return False
        return True

    @staticmethod
    def commander_state():
        """
        Check if commander is initialized
        :return: False if not, True else
        """
        if config.commander is None:
            return False
        else:
            return True

    @staticmethod
    def rolling_topic():
        """
        Check if you can communicate with move_base and amcl_pose
        + Use rosservice /move_base/get_loggers
        + Node: /move_base
        + Type: roscpp/GetLoggers
        + Args:

        + Use rosservice /amcl_pose/get_loggers
        + Node: /amcl_pose
        + Type: roscpp/GetLoggers
        + Args:
        :return: False if not, True else
        """
        get_loggers_move_base = rospy.ServiceProxy(config.move_base + '/get_loggers', GetLoggers)
        get_loggers_amcl_pose = rospy.ServiceProxy(config.amcl_pose + '/get_loggers', GetLoggers)
        try:
            get_loggers_move_base()
            get_loggers_amcl_pose()
        except rospy.service.ServiceException:
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

    @staticmethod
    def connect():
        """
        GET Method

        ROUTE /connect

        Connect to config.move_group with moveit_commander.MoveGroupCommander
        :return: {"Error": "Can't connect to commander please retry with connect button"}, 404 if it's not possible,
        {"Success": "Connected with commander " + config.move_group}, 200 else
        """
        if config.ronoco_mode == "manipulator":
            try:
                config.commander = MoveGroupCommander(config.move_group, wait_for_servers=20)
            except RuntimeError:
                return {"Error": "Can't connect to commander please retry with connect button"}, 404
            return {"Success": "Connected with commander " + config.move_group}, 200
