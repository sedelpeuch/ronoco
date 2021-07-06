"""
Implementation of the action-bt execute allowing the robot to move to a point
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
from flaskr import behavior


class Execute(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is execute, i.e.
    the movement between the current position and a position given in the constructor parameter.
    """

    def __init__(self, name="Execute", identifiant=None, point=None):
        super(Execute, self).__init__(name)
        self.id = identifiant
        self.point = point
        self.commander = behavior.behavior.commander

    def setup(self, timeout):
        """
        Unused here
        :return: True
        """
        self.logger.debug("  %s [Execute::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal of commander
        """
        self.logger.debug("  %s [Execute::initialise()]" % self.name)

    def update(self):
        """
        Execute commander.go()
        :return:
        """
        self.logger.debug("  %s [Execute::update()]" % self.name)
        point = [[self.point[i][k] for k in self.point[i]] for i in self.point]
        self.commander.set_pose_target(point[0] + point[1])
        self.commander.plan()
        self.commander.go()
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Unused here
        """
        self.logger.debug("  %s [Execute::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
