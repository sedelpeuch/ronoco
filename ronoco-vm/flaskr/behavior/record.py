"""
Implementation of the action-bt execute allowing the robot to move to a point
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

import rospy
from flaskr import behavior
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
        """
        self.logger.debug("  %s [Record::initialise()]" % self.name)
        self.compliance(True)

    def update(self):
        """
        """
        self.logger.debug("  %s [Record::update()]" % self.name)
        r = Recorder()
        r.start_recording()
        rospy.sleep(self.time)
        r.stop_and_save(self.identifiant)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Record::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        self.compliance(False)
