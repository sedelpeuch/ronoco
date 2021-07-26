"""
Definition of a logger for the websocket to be displayed in ronoco-ui
"""
import config


def logger(msg):
    """
    Send a message to the /control_log namespace via socketio's emit method
    The message must be a dictionary with at least one key/value

    Some particular keys will be interpreted and reformatted by ronoco-ui (Debug, Success, Error, Warning, Info)

    The other keys will be displayed in a raw way
    :param msg: the message to send
    """
    config.socketio.emit('control_log', msg, namespace='/control_log')


def debug(msg):
    """
    Sends a message with the debug key if the verbosity level is greater than or equal to 4
    :param msg: a string message
    """
    if config.debug >= 4:
        logger({"Debug": msg})


def info(msg):
    """
    Sends a message with the info key if the verbosity level is greater than or equal to 3
    :param msg: a string message
    """
    if config.debug >= 3:
        logger({"Info": msg})


def warn(msg):
    """
    Sends a message with the warning key if the verbosity level is greater than or equal to 2
    :param msg: a string message
    """
    if config.debug >= 2:
        logger({"Warning": msg})


def error(msg):
    """
    Sends a message with the error key if the verbosity level is greater than or equal to 1
    :param msg: a string message
    """
    if config.debug >= 1:
        logger({"Error": msg})
