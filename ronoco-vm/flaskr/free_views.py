from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

import rospy
from std_srvs.srv import SetBool

bp = Blueprint('free_views', __name__, url_prefix='/free')

activate = "False"

@bp.route('/', methods=['GET', 'POST'])
def compliant():
    """
    GET/POST Method allows you to get or set robot compliance
    @return: {'activate':True/False} is e
    """
    global activate
    if request.method == 'POST':
        data = request.get_json()
        try:
            activate = data['activate']
        except KeyError:
            raise BadRequest()
        compliance = rospy.ServiceProxy('set_compliant', SetBool)
        if activate == "True":
            compliance(True)
        elif activate == "False":
            compliance(False)
        else:
            raise BadRequest()
        return {'activate': activate}, 200
    if request.method == 'GET':
        return {'activate': activate}, 200

