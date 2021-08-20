"""
Implementation of the action-bt execute allowing the robot to move to a point
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import config
import logger
import py_trees

class Execute(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is execute, i.e.
    the movement between the current position and a position given in the constructor parameter.
    """

    def __init__(self, name="Execute", point=None):
        super(Execute, self).__init__(name)
        self.point = point
        self.commander = config.commander

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Execute::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal of commander
        """
        self.logger.debug("  %s [Execute::initialise()]" % self.name)
        point = [[self.point[i][k] for k in self.point[i]] for i in self.point]
        self.commander.set_pose_target(point[0] + point[1])

    def update(self):
        """
        Execute commander.go()
        :return:
        """
        self.logger.debug("  %s [Execute::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        result = self.commander.go()
        if not result:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Execute::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
