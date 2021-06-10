from flask import Blueprint, request

import rospy
from std_srvs.srv import SetBool

bp = Blueprint('free_views', __name__, url_prefix='/free')


@bp.route('/')
def compliant():
    set_compliant = rospy.ServiceProxy('set_compliant', SetBool)
    set_compliant(True)
    return '', 200
    

@bp.route('', methods=['POST'])
def set_compliant():
    error = None
    if request.method == 'POST':
        data = request.get_json()
        activate = data['activate']
        if not activate:
            error = 'Activate bool is required'
        if error is None:
            compliance = rospy.ServiceProxy('set_compliant', SetBool)
            if activate:
                compliance(True)
            else:
                compliance(False)
    return '', 200

if __name__=="__main__":
    compliant()