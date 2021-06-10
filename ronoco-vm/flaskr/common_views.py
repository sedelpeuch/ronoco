from flask import (
    Blueprint
)

bp = Blueprint('common_views', __name__, url_prefix='/')


@bp.route('/', methods=('GET', 'POST'))
def index():
    return 'Hello World !'
