"""
Implementation of the action-bt execute allowing the robot to move to a point
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees

from flaskr import behavior
from recorder import Player


class Replay(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behavior.Behavior allowing to define a new behavior. The behaviour is execute, i.e.
    the movement between the current position and a position given in the constructor parameter.
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
        """
        self.logger.debug("  %s [Replay::initialise()]" % self.name)

    def update(self):
        """
        """
        self.logger.debug("  %s [Replay::update()]" % self.name)
        # This returns a moveit_msgs/RobotTrajectory object representing the recorded trajectory
        my_motion = self.player.load(self.identifiant)

        # Go to the start position before replaying the motion
        self.commander.set_joint_value_target(my_motion.joint_trajectory.points[0].positions)
        self.commander.go()

        # Replay the exact same motion
        self.commander.execute(my_motion)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Replay::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
