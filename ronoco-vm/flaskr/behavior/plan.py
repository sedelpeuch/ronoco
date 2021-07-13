"""
Implementation of the action-bt plan allowing the robot to plan to a point
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

from flaskr import behavior


class Plan(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is plan, i.e.
    the planning movement between the current position and a position given in the constructor parameter.
    """

    def __init__(self, name="Plan", point=None):
        super(Plan, self).__init__(name)
        self.point = point
        self.commander = behavior.behavior.commander

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Plan::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal of commander
        """
        self.logger.debug("  %s [Plan::initialise()]" % self.name)
        point = [[self.point[i][k] for k in self.point[i]] for i in self.point]
        self.commander.set_pose_target(point[0] + point[1])

    def update(self):
        """
        Execute commander.plan()
        :return:
        """
        self.logger.debug("  %s [Plan::update()]" % self.name)
        result = self.commander.plan()
        if not result:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Plan::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
