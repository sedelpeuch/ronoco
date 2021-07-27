"""
Implementation of the action-bt replay allowing the robot to replay a previously recorded trajectory
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

import behavior
import logger
from recorder import Player


class Replay(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is replay, i.e.
    replays a previously recorded path.
    """

    def __init__(self, name="Replay", data=None):
        super(Replay, self).__init__(name)
        self.identifiant = data
        self.commander = behavior.behavior.commander
        self.player = Player()

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Replay::setup()]" % self.name)
        return True

    def initialise(self):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Replay::initialise()]" % self.name)

    def update(self):
        """
        Load a recorded motion then execute it
        """
        self.logger.debug("  %s [Replay::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        # This returns a moveit_msgs/RobotTrajectory object representing the recorded trajectory
        my_motion = self.player.load(self.identifiant)
        if my_motion is None:
            return py_trees.common.Status.FAILURE
        # Go to the start position before replaying the motion
        self.commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
        result = self.commander.go()
        if not result:
            return py_trees.common.Status.FAILURE
        # Replay the exact same motion
        result = self.commander.execute(my_motion)
        if not result:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Replay::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
