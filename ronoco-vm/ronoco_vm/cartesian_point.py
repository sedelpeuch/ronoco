"""
This file implements point endpoints to add/ delete and get a cartesian point from compliant mode or rviz
"""
import time

from flask import Blueprint, request

import config
import rospy
import topic_callback
from geometry_msgs.msg import Pose
from moveit_commander import MoveItCommanderException
from visualization_msgs.msg import Marker


class CartesianPoint:
    """
    Allows to add, get and delete an cartesian point in the ros parameters server from Rviz or compliant mode
    """
    id = 0
    cartesianPoints = {}

    def __init__(self):
        self.bp = Blueprint('cartesian_point', __name__, url_prefix='/point')

        self.bp.route('/add/simulation', methods=['POST'])(self.add_point_simulation)
        if config.ronoco_mode == "manipulator":
            self.bp.route('/add/actual', methods=['POST'])(self.manipulator_add_point_real)
        elif config.ronoco_mode == "rolling":
            self.bp.route('/add/actual', methods=['POST'])(self.rolling_add_point_real)
        self.bp.route('/delete/<identifiant>', methods=['POST'])(self.delete_one_point)
        self.bp.route('/delete', methods=['POST'])(self.delete_all_points)
        self.bp.route('/get/<identifiant>', methods=['GET'])(self.get_one_point)
        self.bp.route('/get', methods=['GET'])(self.get_all_points)
        self.publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        try:
            self.cartesianPoints = rospy.get_param("cartesianPoints")
        except KeyError:
            pass
        self.marker_id = len(self.cartesianPoints)
        self.first_marker = True
        self.id = len(self.cartesianPoints)

    def add_bd(self, cartesian_point):
        """
        This method adds a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method is
        called when a PUT request is made on point/add/rviz or point/add/free
        :param cartesian_point: a cartesian point eg{"position": {"x": pose.x, "y": pose.y, "z": pose.z},
                    "orientation": {"x": orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}}
        :return: True if everything is ok
        """
        try:
            self.cartesianPoints = rospy.get_param("cartesianPoints")
        except KeyError:
            pass
        self.cartesianPoints[str(self.id)] = cartesian_point
        rospy.set_param("cartesianPoints", self.cartesianPoints)
        self.id += 1
        return True

    def delete_bd(self, identifiant):
        """
        This method delete a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method
        is called when a DELETE request is made on point/delete
        :param identifiant: a number
        :return: True if point is deleted, False if point doesn't exist
        """
        try:
            self.cartesianPoints = rospy.get_param("cartesianPoints")
            self.cartesianPoints.pop(identifiant)
            rospy.set_param("cartesianPoints", self.cartesianPoints)
        except KeyError:
            return False
        return True

    def find_db(self, identifiant):
        """
        This method find a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method
        is called when a GET request is made on point/get/<number> or point/get
        :param identifiant: a number
        :return: True if point is found, False if point doesn't exist
        """
        try:
            self.cartesianPoints = rospy.get_param("cartesianPoints")
            point = self.cartesianPoints[str(identifiant)]
        except KeyError:
            return False, {}
        return True, point

    def clear_db(self):
        """
        This method delete all cartesian points in the ros parameters server (on the name "cartesianPoints"). This
        method is called when a POST request is made on point/delete
        :return: None
        """
        self.cartesianPoints = {}
        self.id = 0
        rospy.set_param("cartesianPoints", self.cartesianPoints)

    def manipulator_add_point_real(self):
        """
        POST Method

        ROUTE /point/add/actual

        POST body
        {
        }

        Manipulator only

        Allows you to add a Cartesian point corresponding to the current position of the robot.

        The id of the position is automatically given

        :return: id for new cartesian point if everything is ok, a 409 error else
        """
        try:
            current_point = config.commander.get_current_pose()
        except MoveItCommanderException:
            return {"Error": "MoveIt doesn't send response"}, 408
        except AttributeError:
            return {"Error": "Can't connect to commander please retry with connect button"}, 404
        cartesian_point = {
            'position': {
                'x': current_point.pose.position.x,
                'y': current_point.pose.position.y,
                'z': current_point.pose.position.z,
            },
            'orientation': {
                'x': current_point.pose.orientation.x,
                'y': current_point.pose.orientation.y,
                'z': current_point.pose.orientation.z,
                'w': current_point.pose.orientation.w

            }
        }
        if request.method == 'POST':
            self.add_bd(cartesian_point)
            self.marker_add_point(cartesian_point)
            return {"Success": "Add cartesian point with id:" + str(self.id - 1)}, 200

    def add_point_simulation(self):
        """
        POST Method

        ROUTE /point/add/simulation

        POST body
        {
        }

        Allows you to add a Cartesian point corresponding to the current position of the robot in Rviz.

        The id of the position is automatically given

        :return: id for new cartesian point if everything is ok, a 408 error else
        """
        if request.method == 'POST':
            time.sleep(1)
            if topic_callback.position_simulation == {}:
                return {"Error": "Rviz doesn't send response"}, 408
            self.add_bd(topic_callback.position_simulation)
            self.marker_add_point(topic_callback.position_simulation)
            return {"Success": "Add cartesian point with id:" + str(self.id - 1)}, 200

    def rolling_add_point_real(self):
        """
        POST Method

        ROUTE /point/add/actual

        POST body
        {
        }

        Allows you to add a Cartesian point corresponding to the current position of robot.

        The id of the position is automatically given

        :return: id for new cartesian point if everything is ok, a 408 error else.
        """
        if request.method == 'POST':
            time.sleep(1)
            if topic_callback.position_amcl == {}:
                return {"Error": "Amcl doesn't send response"}, 408
            self.add_bd(topic_callback.position_amcl)
            self.marker_add_point(topic_callback.position_amcl)
            return {"Success": "Add cartesian point with id:" + str(self.id - 1)}, 200

    def get_all_points(self):
        """
        GET Method

        ROUTE /point/get

        Allows you to get all Cartesian points in the ros parameters server (on the name "cartesianPoints")

        :return: a json with cartesian points if everything is ok, a 404 error else
        """
        if request.method == 'GET':
            try:
                self.cartesianPoints = rospy.get_param("cartesianPoints")
            except KeyError:
                return {"Error": "No point have been recorded"}, 404
            if self.cartesianPoints == {}:
                return {"Error": "No point have been recorded"}, 404
            return self.cartesianPoints, 200

    def get_one_point(self, identifiant):
        """
        GET Method

        ROUTE /point/get/<identifiant>

        Allows you to get one Cartesian point in the ros parameters server (on the name "cartesianPoints")

        :return: a json with cartesian point it exists, a 404 error else
        """
        if request.method == 'GET':
            state, result = self.find_db(identifiant)
            if state:
                return result, 200
            else:
                return {"Error": "No point have been recorded with id : " + str(identifiant)}, 404

    def delete_one_point(self, identifiant):
        """
        POST Method

        ROUTE /point/delete/<identifiant>

        POST body
        {
        }

        Allows you to delete one Cartesian point in the ros parameters server (on the name "cartesianPoints")

        :return: a response 200 if the point have been deleted, a 404 error else
        """
        if request.method == 'POST':
            status = self.delete_bd(identifiant)
            if status:
                self.marker_delete_point(identifiant)
                return {"Success": "Point have been deleted"}, 200
            else:
                return {"Error": "No point match with id: " + str(identifiant)}, 404

    def delete_all_points(self):
        """
        POST Method

        ROUTE /point/delete

        POST body
        {
        }

        Allows you to delete all Cartesian points in the ros parameters server (on the name "cartesianPoints")

        :return: a response 200
        """
        if request.method == 'POST':
            self.clear_db()
            self.marker_clear()
            return {"Success": "All points have been deleted"}, 200

    def marker_add_point(self, point):
        """
        Add marker in rviz
        :param point: localization of marker
        """
        if self.first_marker:
            self.marker_clear()
            self.first_marker = False
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        if config.ronoco_mode == "rolling":
            marker.ns = config.namespace
        marker.id = self.marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = Pose()
        marker.pose.position.x = point['position']['x']
        marker.pose.position.y = point['position']['y']
        marker.pose.position.z = point['position']['z']
        marker.pose.orientation.x = point['orientation']['x']
        marker.pose.orientation.y = point['orientation']['y']
        marker.pose.orientation.z = point['orientation']['z']
        marker.pose.orientation.w = point['orientation']['w']
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        if config.ronoco_mode == "manipulator":
            marker.header.frame_id = "base_link"
            marker.scale.z = 0.1
        elif config.ronoco_mode == "rolling":
            marker.header.frame_id = "map"
            marker.scale.z = 0.2
        marker.text = str(self.marker_id)
        self.marker_id += 1
        self.publisher.publish(marker)

    def marker_delete_point(self, id):
        """
        Delete one marker on rviz
        :param id: id of marker
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        if config.ronoco_mode == "rolling":
            marker.ns = config.namespace
        marker.id = int(id)
        marker.action = Marker.DELETE
        self.publisher.publish(marker)

    def marker_clear(self):
        """
        Clear all marker in rviz
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        if config.ronoco_mode == "rolling":
            marker.ns = config.namespace
        marker.action = Marker.DELETEALL
        self.marker_id = 0
        self.publisher.publish(marker)
