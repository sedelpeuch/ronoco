"""
Implementation of the action-bt record allowing the robot to record a trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

import rospy
from flaskr import behavior
from flaskr import logger
from recorder import Recorder
from std_srvs.srv import SetBool


class Record(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is execute, i.e.
    the movement between the current position and a position given in the constructor parameter.
    """

    def __init__(self, name="Record", data=None):
        super(Record, self).__init__(name)
        self.identifiant = data['identifiant']
        self.time = int(data['time'])
        self.commander = behavior.behavior.commander
        self.compliance = rospy.ServiceProxy('set_compliant', SetBool)

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Record::setup()]" % self.name)
        return True

    def initialise(self):
        """
        Set robot compliance True
        """
        self.logger.debug("  %s [Record::initialise()]" % self.name)
        self.compliance(True)

    def update(self):
        """
        Start the recording of a trajectory during a specific time then save it
        """
        self.logger.debug("  %s [Record::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        logger.info("Start of recording")
        r = Recorder()
        result = r.start_recording()
        if not result:
            return py_trees.common.Status.FAILURE
        rospy.sleep(self.time)
        result = r.stop_and_save(self.identifiant)
        if not result:
            return py_trees.common.Status.FAILURE
        logger.info("End of recording")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Set robot compliance False
        """
        self.logger.debug("  %s [Record::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        self.compliance(False)
