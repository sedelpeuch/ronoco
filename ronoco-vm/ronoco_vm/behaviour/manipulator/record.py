"""
Implementation of the action-bt record allowing the robot to record a trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import config
import logger
import py_trees
from recorder import Recorder

import rospy
from std_srvs.srv import SetBool


class Record(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. The behaviour is record, i.e.
    record all trajectories during a time specified by a parameter in constructor
    """

    def __init__(self, name="Record", data=None):
        super(Record, self).__init__(name)
        self.identifier = data['identifier']
        self.time = int(data['time'])
        self.commander = config.commander
        self.compliance = rospy.ServiceProxy('set_compliant', SetBool)

    def set_compliance(self, bool):
        if config.mode is None or config.mode == "manual":
            return
        else:
            self.compliance(bool)

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
        self.set_compliance(True)

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
        result = r.stop_and_save(self.identifier)
        if not result:
            return py_trees.common.Status.FAILURE
        logger.info("End of recording")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        Set robot compliance False
        """
        self.logger.debug("  %s [Record::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        self.set_compliance(False)
