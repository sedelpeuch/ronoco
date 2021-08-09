"""
Implementation of the action-bt cartesian allowing the robot to move to a point with a cartesian trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import ast

import config
import logger
import py_trees

import rospy
import rosservice


class EndEffector(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is the activation
    or deactivation of the gripper using the service provided as a parameter to ronoco.launch.
    """

    def __init__(self, name="EndEffector", data=None):
        super(EndEffector, self).__init__(name)
        self.parameters = ast.literal_eval(data)
        self.service_type = rosservice.get_service_type("/" + config.end_effector).replace('/', '.').rsplit(".", 1)
        self.service = None

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [EndEffector::setup()]" % self.name)
        imported = getattr(__import__(self.service_type[0] + ".srv", fromlist=[self.service_type[1]]),
                           self.service_type[1])
        self.service = rospy.ServiceProxy("/" + config.end_effector, imported)
        return True

    def initialise(self):
        """
        No specific treatment
        """
        self.logger.debug("  %s [EndEffector::initialise()]" % self.name)

    def update(self):
        """
        Compute cartesian path, then if the reliability is sufficient, execute the trajectory
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [EndEffector::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        try:
            result = self.service(*self.parameters)
        except rospy.ServiceException:
            return py_trees.common.Status.FAILURE
        if str(result) == "error: 255":
            logger.warn("The effector has caught something")
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [EndEffector::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
