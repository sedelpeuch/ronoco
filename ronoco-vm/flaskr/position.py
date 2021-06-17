"""
This file implements position endpoints to record the blue sphere current position
"""
import time

from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

import rospy
from flaskr import common
from moveit_commander.move_group import MoveGroupCommander
from . import topic_callback


class Position():

    def __init__(self):
        self.bp = Blueprint('position_view', __name__, url_prefix='/position')

        self.commander = MoveGroupCommander("arm_and_finger", wait_for_servers=20)
        self.bp.route('/record/rviz', methods=['GET', 'POST'])(self.position_from_rviz)
        self.bp.route('/record/free', methods=['GET', 'POST'])(self.position_from_free_mode)
        # self.bp.route('/get', methods=['GET'])(self.)

    def position_from_free_mode(self):
        if common.Common().robot_state()['robot_state'] == True:
            current_position = self.commander.get_current_pose()
            position = {
                'position': {
                    'x': current_position.pose.position.x,
                    'y': current_position.pose.position.y,
                    'z': current_position.pose.position.z,
                },
                'orientation': {
                    'x': current_position.pose.orientation.x,
                    'y': current_position.pose.orientation.y,
                    'z': current_position.pose.orientation.z,
                    'w': current_position.pose.orientation.w

                }
            }
            if request.method == 'GET':
                return position, 200
            if request.method == 'POST':
                data = request.get_json()
                print(data)
                try:
                    name = data['name']
                except KeyError:
                    raise BadRequest()
                except TypeError:
                    raise BadRequest()
                try:
                    rospy.set_param("position/" + name, position)
                    return "Add point" + name, 200
                except TypeError:
                    return "Name must be a string", 400
            return position
        else:
            return "Robot is not compliant", 409

    @staticmethod
    def position_from_rviz():
        begin = time.time()
        while topic_callback.position == {}:
            if time.time() - begin > 10:
                return "Rviz doesn't send response", 408
            rospy.loginfo("Waiting position from Rviz")
        if request.method == 'GET':
            return topic_callback.position, 200
        elif request.method == 'POST':
            data = request.get_json()
            try:
                name = data['name']
            except KeyError:
                raise BadRequest()
            except TypeError:
                raise BadRequest()
            rospy.set_param("position/" + name, topic_callback.position)
            return "Add point" + name, 200
        return topic_callback.position

    # @staticmethod
    # def get_all_position():
    #     if request.method=='GET':
    #
