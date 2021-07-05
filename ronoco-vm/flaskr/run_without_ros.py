"""
This file is the entry point of flask api without ros.
Launch python3 flaskr/run_without_ros.py to run flask server.

WARN : This way of launching the server is for development only and must not be used to control a robot with ros
"""

import os

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
from werkzeug.debug import DebuggedApplication


class ronoco_vm:
    """
    Define and setup flask server and ros topic suscriber / publisher for ronoco-vm
    """

    def __init__(self):
        """
        Launch flask server when ronoco_vm is created (this constructor uses SocketIO)
        """

        self.create_app()
        socketio = SocketIO(self.app, logger=True)
        self.app.logger.warning("This way of launching the server is for development only and must not be used to "
                                "control a robot with ros")
        socketio.run(self.app)

    def create_app(self, test_config=None):
        """
        Build a Flask instance and configure it
        :param test_config: path to configuration file (Défault : None)
        :return: a Flask instance
        """
        # create and configure the app
        self.app = Flask(__name__, instance_relative_config=True)
        self.app.config.from_mapping(
            SECRET_KEY='dev',
        )
        self.app.debug = True
        self.app.wsgi_app = DebuggedApplication(self.app.wsgi_app, evalex=True)

        if test_config is None:
            # load the instance config, if it exists, when not testing
            self.app.config.from_pyfile('config.py', silent=True)
        else:
            # load the test config if passed in
            self.app.config.from_mapping(test_config)
        try:
            os.makedirs(self.app.instance_path)
        except OSError:
            pass
        self.setup_app()

    def setup_app(self):
        """
        Register blueprint in app.

        The class attribute "app" must contain an Flask instance
        :return: None
        """
        from flaskr import play
        self.app.register_blueprint(play.Play().bp)

        from flaskr import common
        self.app.register_blueprint(common.Common().bp)

        from flaskr import free
        self.app.register_blueprint(free.bp)

        CORS(self.app)


if __name__ == "__main__":
    ronoco_vm()
