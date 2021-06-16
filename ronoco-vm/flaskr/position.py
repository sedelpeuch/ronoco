"""
This file implements position endpoints to record the blue sphere current position
"""
import json
import time

import flask
from flask import Blueprint, request
from flask_cors import cross_origin
from werkzeug.exceptions import BadRequest, NotFound
from . import topic_callback

import rospy
from std_srvs.srv import SetBool
from visualization_msgs.msg import InteractiveMarkerUpdate
from . import common

bp = Blueprint('position_view', __name__, url_prefix='/position')

@bp.route('/', methods=['POST', 'GET'])
def record():
    return str(topic_callback.position), 200
