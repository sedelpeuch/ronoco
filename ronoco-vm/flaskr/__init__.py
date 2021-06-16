import os

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO

import rospy


def create_app(test_config=None):
    # create and configure the app
    app = Flask(__name__, instance_relative_config=True)
    app.config.from_mapping(
        SECRET_KEY='dev',
        DATABASE=os.path.join(app.instance_path, 'flaskr.sqlite'),
    )

    if test_config is None:
        # load the instance config, if it exists, when not testing
        app.config.from_pyfile('config.py', silent=True)
    else:
        # load the test config if passed in
        app.config.from_mapping(test_config)

    # ensure the instance folder exists
    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    from flaskr import common_views
    app.register_blueprint(common_views.bp)

    from flaskr import free_views
    app.register_blueprint(free_views.bp)

    from flaskr import position_views
    app.register_blueprint(position_views.bp)
    CORS(app)
    return app


if __name__ == "__main__":
    app = create_app()
    socketio = SocketIO(app)
    rospy.init_node('user')
    rospy.loginfo("User node is serving the Web app")
    socketio.run(app)
    rospy.spin()
