"""
This behaviour allows you to stop execution of behaviour tree during n secondes
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import logger
import py_trees

import rospy


class Sleep(py_trees.behaviour.Behaviour):
    """
    This behaviour allows you to stop execution of behaviour tree during n secondes
    """

    def __init__(self, name, data):
        super(Sleep, self).__init__(name)
        self.time = float(data)

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Sleep::setup()]" % self.name)
        return True

    def initialise(self):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Sleep::initialise()]" % self.name)

    def update(self):
        """
        Executes the service with the list of associated parameters
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Sleep::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        rospy.sleep(self.time)
        return py_trees.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Sleep::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
