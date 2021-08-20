"""
This file implements free endpoints to set and get the compliance (or 0 gravity robot WIP) of robot
"""

from flask import Blueprint, request
from werkzeug.exceptions import BadRequest, NotFound

import common
import config
import rospy
import rosservice

bp = Blueprint('free_endpoint', __name__, url_prefix='/free')

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
    if common.Common().ros_state() and config.ronoco_mode == "manipulator":
        global compliant
        if request.method == 'POST':
            if config.mode == None:
                return {"Warning": "compliant mode is None"}, 200
            if config.mode == "manual":
                return {'Info': "compliant mode is manual"}, 200
            data = request.get_json()
            try:
                compliant = data['compliant']
            except KeyError:
                raise BadRequest()
            except TypeError:
                raise BadRequest()
            service_type = rosservice.get_service_type("/" + config.mode).replace('/', '.').rsplit(".", 1)
            imported = getattr(__import__(service_type[0] + ".srv", fromlist=[service_type[1]]), service_type[1])
            compliance = rospy.ServiceProxy(config.mode, imported)
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
