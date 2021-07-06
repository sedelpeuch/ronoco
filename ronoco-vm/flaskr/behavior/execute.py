#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import py_trees
from flaskr import behavior


class Execute(py_trees.behaviour.Behaviour):
    def __init__(self, name="Execute", id=None, point=None):
        super(Execute, self).__init__(name)
        self.id = id
        self.point = point
        self.commander = behavior.behavior.commander

    def setup(self, timeout):
        self.logger.debug("  %s [Execute::setup()]" % self.name)
        return True

    def initialise(self):
        self.logger.debug("  %s [Execute::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Execute::update()]" % self.name)
        point = [[self.point[i][k] for k in self.point[i]] for i in self.point]
        self.commander.set_pose_target(point[0] + point[1])
        self.commander.go()
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug("  %s [Execute::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
