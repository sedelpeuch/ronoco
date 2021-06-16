"""
This file implements position endpoints to record the blue sphere current position
"""
import json
import time

import flask
from flask import Blueprint, request
from flask_cors import cross_origin
from werkzeug.exceptions import BadRequest, NotFound

import rospy
from std_srvs.srv import SetBool
from visualization_msgs.msg import InteractiveMarkerUpdate
from . import common_views

bp = Blueprint('position_view', __name__, url_prefix='/position')

glob = {}


@bp.route('/', methods=['POST', 'GET'])
def record():
    response = rospy.Subscriber(
        "/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update",
        InteractiveMarkerUpdate, callback)
    return str(glob), 200


def callback(data):
    global glob
    glob = data
