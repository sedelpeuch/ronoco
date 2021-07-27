"""
Implementation of the action-bt cartesian allowing the robot to move to a point with a cartesian trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

import behavior
import geometry_msgs.msg
import logger


class Cartesian(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is cartesian path,
    i.e. the cartesian movement between the current position and a position given in the constructor parameter .
    """

    def __init__(self, name="Cartesian", data=None):
        super(Cartesian, self).__init__(name)
        self.point = data['point']
        self.reliability = data['reliability']
        self.eef = data['eef']
        self.commander = behavior.behavior.commander
        self.wpose = None

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Cartesian::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal as a PoseStamped()
        """
        self.logger.debug("  %s [Cartesian::initialise()]" % self.name)
        self.wpose = geometry_msgs.msg.PoseStamped()
        self.wpose.pose.position.x = self.point['position']['x']
        self.wpose.pose.position.y = self.point['position']['y']
        self.wpose.pose.position.z = self.point['position']['z']
        self.wpose.pose.orientation.x = self.point['orientation']['x']
        self.wpose.pose.orientation.y = self.point['orientation']['y']
        self.wpose.pose.orientation.z = self.point['orientation']['z']
        self.wpose.pose.orientation.w = self.point['orientation']['w']

    def update(self):
        """
        Compute cartesian path, then if the reliability is sufficient, execute the trajectory
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Cartesian::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        plan, fraction = self.commander.compute_cartesian_path([self.wpose.pose], float(self.eef), 0.0)
        if fraction <= int(self.reliability) / 100:
            logger.warn("The Cartesian path has a lower reliability than required")
            return py_trees.common.Status.FAILURE
        result = self.commander.execute(plan)
        if not result:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Cartesian::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
