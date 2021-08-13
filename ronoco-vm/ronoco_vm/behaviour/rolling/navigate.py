"""
Implementation of the action-bt cartesian allowing the robot to move to a point with a cartesian trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import config
import py_trees

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Header


class Navigate(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is cartesian path,
    i.e. the cartesian movement between the current position and a position given in the constructor parameter .
    """

    def __init__(self, name="Navigate", data=None):
        super(Navigate, self).__init__(name)
        self.point = data
        self.publisher = rospy.Publisher(config.move_base + '/goal', MoveBaseActionGoal)
        self.goal = None
        self.header = None

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Navigate::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal as a PoseStamped()
        """
        self.logger.debug("  %s [Navigate::initialise()]" % self.name)
        self.goal = MoveBaseActionGoal()
        self.header = Header()
        self.header.stamp = rospy.Time.now()
        self.header.frame_id = 'map'
        self.goal.goal.target_pose.header = self.header
        self.goal.goal.target_pose.pose.position.x = self.point['position']['x']
        self.goal.goal.target_pose.pose.position.y = self.point['position']['y']
        self.goal.goal.target_pose.pose.position.z = self.point['position']['z']
        self.goal.goal.target_pose.pose.orientation.x = self.point['orientation']['x']
        self.goal.goal.target_pose.pose.orientation.y = self.point['orientation']['y']
        self.goal.goal.target_pose.pose.orientation.z = self.point['orientation']['z']
        self.goal.goal.target_pose.pose.orientation.w = self.point['orientation']['w']

    def update(self):
        """
        Compute cartesian path, then if the reliability is sufficient, execute the trajectory
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Navigate::update()]" % self.name)
        self.publisher.publish(self.goal)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Navigate::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
