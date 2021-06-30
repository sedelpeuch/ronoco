"""
This file implements free endpoints to set and get the compliance (or 0 gravity robot WIP) of robot
"""

from flask import Blueprint, request
from werkzeug.exceptions import BadRequest, NotFound

import rospy
from flaskr import common
from std_srvs.srv import SetBool

bp = Blueprint('free_views', __name__, url_prefix='/free')

# Global variable to record compliant state of robot
compliant = "False"


@bp.route('/', methods=['GET', 'POST'])
def free():
    """
    GET/POST Method

    ROUTE /free/

    POST body
    {
    "compliant" : "True"/"False"
    }

    Allows you to get or set robot compliance
        + Use rosservice /set_compliant
        + Node: /joint_trajectory_action_server
        + Type: std_srvs/SetBool
        + Args: data
    :return: if everything is ok : {"compliant":"True"/"False"} else an HttpError
    """
    if common.Common().robot_state()['robot_state']:
        global compliant
        if request.method == 'POST':
            data = request.get_json()
            try:
                compliant = data['compliant']
            except KeyError:
                raise BadRequest()
            except TypeError:
                raise BadRequest()
            compliance = rospy.ServiceProxy('set_compliant', SetBool)
            if compliant == "True":
                compliance(True)
            elif compliant == "False":
                compliance(False)
            else:
                raise BadRequest()
            return {'compliant': compliant}, 200
        elif request.method == 'GET':
            return {'compliant': compliant}, 200
    else:
        raise NotFound()
