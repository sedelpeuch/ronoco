"""
Patrolling behaviour, i.e. navigation on the edges of a user-defined polygon
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import time
from math import atan2

import config
import py_trees
import logger
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseGoal
from ronoco_vm.coverage.libs.list_helper import list_avg_dist
from ronoco_vm.coverage.libs.marker_visualization import MarkerVisualization
from std_msgs.msg import Header


class Patrol(py_trees.behaviour.Behaviour, MarkerVisualization):
    """
    Patrolling behaviour, i.e. navigation on the edges of a user-defined polygon
    """

    def __init__(self, name, data):
        MarkerVisualization.__init__(self)
        super(Patrol, self).__init__(name)
        self.commander = config.commander
        self.subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.rvizPointReceived)
        self.publisher = rospy.Publisher('/clicked_point', PointStamped)
        self.thread = None
        self.lClickPoints = []
        self.points = data

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Patrol::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Initialize thread
        """
        self.logger.debug("  %s [Patrol::initialise()]" % self.name)
        self.thread = threading.Thread(target=self.publish_points)

    def update(self):
        """
        Starts the thread for publishing points if the user has filled the point table via Node-RED. Then waits for the end of the navigation execution
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Patrol::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        if self.points != '':
            self.thread.start()
        while config.patrol == py_trees.Status.RUNNING or config.patrol is None:
            time.sleep(1)
        return config.patrol

    def terminate(self, new_status):
        """
        Set config patrol to None
        """
        self.logger.debug("  %s [Patrol::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        config.patrol = None
        self.subscriber.unregister()

    def rvizPointReceived(self, point):
        """
        Handler of the topic clicked_point allowing to add a point to the current polygon and if the point closes the polygon to start the patrol
        :param point: received point
        """
        self.lClickPoints.append(point)
        points = [(p.point.x, p.point.y) for p in self.lClickPoints]
        self.global_frame = point.header.frame_id
        if len(self.lClickPoints) > 2:
            # All points must have same frame_id
            if len(set([p.header.frame_id for p in self.lClickPoints])) != 1:
                raise ValueError()
            points_x = [p.point.x for p in self.lClickPoints]
            points_y = [p.point.y for p in self.lClickPoints]
            avg_x_dist = list_avg_dist(points_x)
            avg_y_dist = list_avg_dist(points_y)
            dist_x_first_last = abs(points_x[0] - points_x[-1])
            dist_y_first_last = abs(points_y[0] - points_y[-1])
            if dist_x_first_last < avg_x_dist / 10.0 and dist_y_first_last < avg_y_dist / 10.0:
                # last point is close to maximum, construct polygon
                rospy.loginfo("Creating polygon %s" % (str(points)))
                self.visualize_area(points, close=True)
                self.patrol()
                self.visualize_area(points, close=True, show=False)
                self.lClickPoints = []
                return
        self.visualize_area(points, close=False)

    def patrol(self):
        """
        Browse the received points and ask the SimpleActionServer to go there
        """
        config.patrol = py_trees.Status.RUNNING
        for point in range(len(self.lClickPoints)):
            try:
                last_point = self.lClickPoints[point - 1]
            except IndexError:
                last_point = self.lClickPoints[0]
            try:
                next_point = self.lClickPoints[point + 1]
            except IndexError:
                next_point = self.lClickPoints[0]

            # Previous
            dx = self.lClickPoints[point].point.x - last_point.point.x
            dy = self.lClickPoints[point].point.y - last_point.point.y
            angle = atan2(dy, dx)
            angle_quat_last = tf.transformations.quaternion_from_euler(0, 0, angle)

            # Next
            dx = next_point.point.x - self.lClickPoints[point].point.x
            dy = next_point.point.y - self.lClickPoints[point].point.y
            angle = atan2(dy, dx)
            angle_quat_next = tf.transformations.quaternion_from_euler(0, 0, angle)
            self.go_to(point, angle_quat_last)
            self.go_to(point, angle_quat_next)

        config.patrol = py_trees.Status.SUCCESS

    def go_to(self, point, angle):
        """
        Generic function allowing to go to a point (x,y,z) according to an angle
        :param point: position goal (x,y,z)
        :param angle: orientation goal (x,y,z,w)
        :return:
        """
        goal = MoveBaseGoal()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        goal.target_pose.header = header
        goal.target_pose.pose.position.x = self.lClickPoints[point].point.x
        goal.target_pose.pose.position.y = self.lClickPoints[point].point.y
        goal.target_pose.pose.position.z = self.lClickPoints[point].point.z
        goal.target_pose.pose.orientation.x = angle[0]
        goal.target_pose.pose.orientation.y = angle[1]
        goal.target_pose.pose.orientation.z = angle[2]
        goal.target_pose.pose.orientation.w = angle[3]
        self.commander.send_goal(goal)

        self.commander.wait_for_result()
        state = self.commander.get_state()
        if state != GoalStatus.SUCCEEDED:
            config.patrol = py_trees.Status.FAILURE

    def publish_points(self):
        """
        Allows to publish the points on the clicked point topic in case the user provides a point list in Node-RED
        """
        time.sleep(5)
        while config.patrol != py_trees.Status.RUNNING and config.patrol is not None:
            time.sleep(1)
        self.points.append(self.points[0])
        for i in range(len(self.points)):
            point = PointStamped()
            header = Header()
            header.frame_id = "map"
            header.stamp = rospy.Time.now()
            point.header = header
            point.point.x = self.points[i]['position']['x']
            point.point.y = self.points[i]['position']['y']
            point.point.z = self.points[i]['position']['z']
            self.publisher.publish(point)
