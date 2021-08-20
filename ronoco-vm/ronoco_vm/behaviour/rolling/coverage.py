"""
Implementation of the coverage block allowing to cover the whole surface of a rectangle defined by 4 points
"""
# !/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import time

import config
import logger
import py_trees

import rospy
from geometry_msgs.msg import PointStamped
from ronoco_vm.coverage import path_coverage_node
from std_msgs.msg import Header


class Coverage(py_trees.behaviour.Behaviour):
    """
    Class inherited from py_tree.behaviour.Behavior allowing to define a new behaviour. Implementation of the coverage
    block allowing to cover the whole surface of a rectangle defined by 4 points
    """

    def __init__(self, name, data):
        super(Coverage, self).__init__(name)
        self.map_drive = None
        self.robot_width = float(data['robot_width'])
        self.points = data['points']
        self.publisher = rospy.Publisher('/clicked_point', PointStamped)
        self.thread = None

    def setup(self, timeout):
        """
        No specific treatment
        :return: True
        """
        self.logger.debug("  %s [Coverage::setup()]" % self.name)
        return True

    def initialise(self):
        """
        No specific treatment
        """
        self.logger.debug("  %s [Coverage::initialise()]" % self.name)
        self.thread = threading.Thread(target=self.publish_points)

    def update(self):
        """
        Creates an instance of MapDrive and waits for it to complete its execution to return success
        :return: SUCCESS or FAILURE
        """
        self.logger.debug("  %s [Coverage::update()]" % self.name)
        logger.debug("Execute block " + self.name)
        if self.points != '':
            try:
                self.thread.start()
            except RuntimeError:
                pass
        self.map_drive = path_coverage_node.MapDrive(self.robot_width)
        while config.coverage != py_trees.Status.SUCCESS and config.coverage != py_trees.Status.FAILURE:
            time.sleep(1)
        if config.coverage == py_trees.Status.FAILURE:
            self.map_drive.on_shutdown()
        return config.coverage

    def terminate(self, new_status):
        """
        Replace the coverage value of the configuration file with False
        """
        self.logger.debug("  %s [Coverage::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
        config.coverage = None

    def publish_points(self):
        while config.coverage != py_trees.Status.RUNNING:
            time.sleep(1)
        self.points.append(self.points[0])
        for i in range(len(self.points)):
            point = PointStamped()
            header = Header()
            header.frame_id = "map"
            header.stamp = rospy.Time.now()
            point.header = header
            point.point.x = self.points[i]['position']['x']
            point.point.y = self.points[i]['position']['y']
            point.point.z = self.points[i]['position']['z']
            self.publisher.publish(point)
