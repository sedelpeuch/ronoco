"""
Definition of endpoint teleoperation allowing to manually move rolling robot
"""
import threading

from flask import Blueprint

import config
import rospy
from geometry_msgs.msg import Twist


class Teleoperation:
    """
    Definition of endpoint teleoperation allowing to manually move rolling robot
    """

    def __init__(self):
        self.bp = Blueprint('teleoperation_endpoint', __name__, url_prefix='/teleop')
        self.bp.route('/<direction>', methods=['POST'])(self.set_vel)
        self.publisher = rospy.Publisher(config.cmd_vel, Twist, queue_size=10)
        self.direction_callback = {
            "force-stop": self.force_stop,
            "forward": self.forward,
            "backward": self.backward,
            "left": self.left,
            "right": self.right,
        }
        self.twist = Twist()

    def set_vel(self, direction):
        """
        POST Method

        POST Body
        {}

        ROUTE /teleop/<direction>

        Where direction is :
        - force-stop
        - forward
        - backward
        - left
        - right

        Set a twist vector with desired velocity then launch thread to execute it
        :param direction: desired direction in force-stop, forward, backward, left, right
        :return: "Success", 200
        """
        self.direction_callback[direction]()
        self.thread = threading.Thread(target=self.move)
        self.thread.start()
        return "Success", 200

    def move(self):
        """
        Target of thread which continuously publish actualised twist message on topic cmd_vel
        :return:
        """
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publisher.publish(self.twist)
            r.sleep()

    def forward(self):
        """
        Add 0.01 to twist linear x (forward)
        """
        self.twist.linear.x += 0.01

    def backward(self):
        """
        Remove 0.01 to twist linear x (backward)
        """
        self.twist.linear.x -= 0.01

    def left(self):
        """
        Add 0.1 to twist angular z (left)
        """
        self.twist.angular.z += 0.1

    def right(self):
        """
        Remove 0.1 to twist angular z (right)
        """
        self.twist.angular.z -= 0.1

    def force_stop(self):
        """
        Set to 0.0 twist linear x and twist angular z (stop)
        """
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
