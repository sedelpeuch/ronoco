"""
Implementation of the action-bt cartesian allowing the robot to move to a point with a cartesian trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import ast

import logger
import py_trees

import rospy
import rosservice


class Service(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is the activation
    or deactivation of the gripper using the service provided as a parameter to ronoco.launch.
    """

    def __init__(self, name="Service", data=None):
        super(Service, self).__init__(name)
        self.service_name = data['service_name']
        self.service_type = rosservice.get_service_type("/" + self.service_name).replace('/', '.').rsplit(".", 1)
        self.service_parameter = ast.literal_eval(data['service_parameter'])
        self.service = None

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Service::setup()]" % self.name)
        imported = getattr(__import__(self.service_type[0] + ".srv", fromlist=[self.service_type[1]]),
                           self.service_type[1])
        self.service = rospy.ServiceProxy(self.service_name, imported)
        return True

    def initialise(self):
        """
        Set target goal as a PoseStamped()
        """
        self.logger.debug("  %s [Service::initialise()]" % self.name)

    def update(self):
        """
        Compute cartesian path, then if the reliability is sufficient, execute the trajectory
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Service::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        try:
            result = self.service(*self.service_parameter)
        except rospy.ServiceException:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Service::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
