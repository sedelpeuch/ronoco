from flaskr import config


def logger(msg):
    config.socketio.emit('control_log', msg, namespace='/control_log')


def debug(msg):
    if config.debug >= 4:
        logger({"Debug": msg})


def info(msg):
    if config.debug >= 3:
        logger({"Info": msg})


def warn(msg):
    if config.debug >= 2:
        logger({"Warning": msg})


def error(msg):
    if config.debug >= 1:
        logger({"Error": msg})
