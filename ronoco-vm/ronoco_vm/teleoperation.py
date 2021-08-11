"""

"""
import threading

from flask import Blueprint

import rospy
from geometry_msgs.msg import Twist


class Teleoperation:
    """
    Definition of the endpoint teleoperation
    """

    def __init__(self):
        self.bp = Blueprint('teleoperation_endpoint', __name__, url_prefix='/teleop')
        self.bp.route('/<direction>', methods=['POST'])(self.set_vel)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.direction_callback = {
            "force-stop": self.force_stop,
            "forward": self.forward,
            "backward": self.backward,
            "left": self.left,
            "right": self.right,
        }
        self.twist = Twist()

    def set_vel(self, direction):
        self.direction_callback[direction]()
        self.thread = threading.Thread(target=self.move)
        self.thread.start()
        return "Success", 200

    def move(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publisher.publish(self.twist)
            r.sleep()

    def forward(self):
        self.twist.linear.x += 0.01

    def backward(self):
        self.twist.linear.x -= 0.01

    def left(self):
        self.twist.angular.z += 0.1

    def right(self):
        self.twist.angular.z -= 0.01

    def force_stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
