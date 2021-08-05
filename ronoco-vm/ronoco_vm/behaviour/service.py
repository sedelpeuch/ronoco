"""
This behaviour allows you to call any service by specifying its name and arguments
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
    This behaviour allows you to call any service by specifying its name and arguments
    """

    def __init__(self, name="Service", data=None):
        super(Service, self).__init__(name)
        self.service_name = data['service_name']
        self.service_type = rosservice.get_service_type("/" + self.service_name).replace('/', '.').rsplit(".", 1)
        self.service_parameter = ast.literal_eval(data['service_parameter'])
        self.service = None

    def setup(self, timeout):
        """
        Import the srv associated with the service type and retrieve the activation function with ServiceProxy
        :return: True
        """
        self.logger.debug("  %s [Service::setup()]" % self.name)
        imported = getattr(__import__(self.service_type[0] + ".srv", fromlist=[self.service_type[1]]),
                           self.service_type[1])
        self.service = rospy.ServiceProxy(self.service_name, imported)
        return True

    def initialise(self):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Service::initialise()]" % self.name)

    def update(self):
        """
        Executes the service with the list of associated parameters
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
