"""
Implementation of the action-bt cartesian allowing the robot to move to a point with a cartesian trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import config
import logger
import py_trees

from ronoco_vm.coverage import path_coverage_node


class Coverage(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is cartesian path,
    i.e. the cartesian movement between the current position and a position given in the constructor parameter .
    """

    def __init__(self, name="Coverage", data=0.3):
        super(Coverage, self).__init__(name)
        self.map_drive = None
        self.robot_width = data

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Coverage::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set target goal as a PoseStamped()
        """
        self.logger.debug("  %s [Coverage::initialise()]" % self.name)

    def update(self):
        """
        Compute cartesian path, then if the reliability is sufficient, execute the trajectory
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Coverage::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        self.map_drive = path_coverage_node.MapDrive(self.robot_width)
        while not config.finished:
            time.sleep(1)
        self.map_drive.on_shutdown()
        return py_trees.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Coverage::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        config.finished = False
