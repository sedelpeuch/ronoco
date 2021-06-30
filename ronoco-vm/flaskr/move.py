"""
This file implements move endpoint to plan or execute a trajectory or a cycle between two (or more) points
"""
import time

from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

import rospy
from flaskr import cartesianpoint

execute = True


class Move:
    """
    Allows to plan or execute a trajectory or a cycle between two (or more) points
    """

    def __init__(self):
        self.bp = Blueprint('move_endpoint', __name__, url_prefix='/move')
        self.CartesianPoint = cartesianpoint.CartesianPoint()

        self.bp.route('/', methods=['POST'])(self.move)
        self.bp.route('/stop', methods=['POST'])(self.stop_infinite)

    def move(self):
        """
        POST Method

        ROUTE /move/

        POST body
        {
        "id": list
        "mode": "plan"/"execute"/"infinite"
        }

        Allows you to plan / execute an action between to point recorded in the ros parameters server (on the name
        "cartesianPoints")

        :return: id for new cartesian point if everything is ok, a 409 error else
        """
        if request.method == 'POST':
            data = request.get_json()
            try:
                ids = data['id']
                mode = data['mode']
            except KeyError:
                raise BadRequest()
            except TypeError:
                raise BadRequest()
            states = list()
            results = list()
            if mode not in ["plan", "execute", "infinite"]:
                return {"Error": "Incorrect mode"}, 400
            for i in range(len(ids)):
                find = self.CartesianPoint.find_db(ids[i])
                states.append(find[0])
                results.append(find[1])
            if False in states:
                return {"Error": "Incorrect id"}, 404

            # Check if rviz is alive
            position = {}
            begin = time.time()
            while position == {}:
                if time.time() - begin > 10:
                    return {"Error": "Rviz doesn't send response"}, 408
                rospy.loginfo("Waiting response from Rviz")
                position = self.CartesianPoint.commander.get_current_pose()
            self.switch_mode(results, mode)
            return {"Success": "Action has been realized"}, 200

    @staticmethod
    def dict_to_list(point):
        """
        Transform a dictionary point into a list
        :param point: a dictionary point {'position': {'x':, 'y':, 'z':}, 'orientation': {'x':, 'y':, 'z':, 'w':}}
        """
        return [[
            point['position']['x'],
            point['position']['y'],
            point['position']['z']
        ],
            [
                point['orientation']['x'],
                point['orientation']['y'],
                point['orientation']['z'],
                point['orientation']['w']
            ]]

    def switch_mode(self, points, mode):
        """
        Execute different action functions according to "mode" parameter
        :param points: list of waypoints
        :param mode: plan, execute or infinite
        """
        global execute
        execute = True
        for i in range(len(points)):
            points[i] = self.dict_to_list(points[i])
        if mode == "plan":
            self.plan(points)
        elif mode == "execute":
            self.execute(points)
        elif mode == "infinite":
            while execute:
                self.execute(points)
            self.CartesianPoint.commander.stop()

    def plan(self, points):
        """
        Plan a trajectory in rviz between 2 points
        :param points: list of waypoints
        """
        for p in points:
            self.CartesianPoint.commander.set_pose_target(p[0] + p[1])
            self.CartesianPoint.commander.plan()

    def execute(self, points):
        """
        Execute a trajectory between 2 points
        :param points : list of waypoints
        """
        for p in points:
            self.CartesianPoint.commander.set_pose_target(p[0] + p[1])
            self.CartesianPoint.commander.go()
        self.CartesianPoint.commander.stop()

    def stop_infinite(self):
        global execute
        execute = False
        return "", 200
