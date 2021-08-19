"""
This file implements some constants for behaviour like kind of types, leaves and create an instance of CartesianPoint()
commander
"""
import ast
from collections import Counter

import cartesian_point
import config
import py_trees

import behaviour
import rosservice


def selector(name, data, child):
    return True, py_trees.composites.Selector(), None


def sequence(name, data, child):
    return True, py_trees.composites.Sequence(), None


def parallel(name, data, child):
    return True, py_trees.composites.Parallel(), None


def execute(data, child, name):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if name is None or name == "":
        name = "execute"
    if data is None:
        return False, None, "No data"
    try:
        state, point = cartesian_point.CartesianPoint().find_db(int(data))
    except ValueError:
        return False, None, "Identifier is not an integer"
    if not state:
        return False, None, "Point " + data + " not found"
    return True, behaviour.execute.Execute(name, point), None


def plan(name, data, child):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if data is None:
        return False, None, "No data"
    try:
        state, point = cartesian_point.CartesianPoint().find_db(int(data))
    except ValueError:
        return False, None, "Identifier is not an integer"
    if not state:
        return False, None, "Point " + data + " not found"
    return True, behaviour.plan.Plan(name, point), None


def condition(name, data, child):
    if data is None:
        return False, None, "No data"
    try:
        return True, py_trees.decorators.Condition(name=name, status=states[data], child=child), None
    except KeyError:
        return False, None, "data is not a states"


def inverter(name, data, child):
    try:
        return True, py_trees.decorators.Inverter(child, name), None
    except TypeError:
        return False, None, "Internal error"


def timeout(name, data, child):
    if data is None:
        return False, None, "No data"
    try:
        return True, py_trees.decorators.Timeout(name=name, duration=int(data), child=child), None
    except TypeError:
        return False, None, "Internal error"
    except ValueError:
        return False, None, "Identifier is not an integer"


def cartesian(name, data, child):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if name is None or name == "":
        name = "Cartesian"
    if data is None:
        return False, None, "No data"
    try:
        state, point = cartesian_point.CartesianPoint().find_db(int(data['point_id']))
    except ValueError:
        return False, None, "point_id is not an integer"
    if not state:
        return False, None, "Point " + data + " not found"
    return True, behaviour.cartesian.Cartesian(name,
                                               {"point": point, "reliability": data['reliability'],
                                                "eef": data['eef']}), None


def record(name, data, child):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if name is None or name == "":
        name = "Record"
    if data is None:
        return False, None, "No data"
    return True, behaviour.record.Record(name,
                                         {"identifier": data['identifier'], "time": data['time']}), None


def replay(name, data, child):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if name is None or name == "":
        name = "Replay"
    if data is None:
        return False, None, "No data"
    return True, behaviour.replay.Replay(name, data), None


def end_effector(name, data, child):
    if config.mode != "manipulator":
        return False, None, "Need manipulator mode"
    if name is None or name == "":
        name = "end effector"
    if data is None:
        return False, None, "No data"
    if rosservice.get_service_type("/" + config.end_effector) is None:
        return False, None, "Service not found"
    return True, behaviour.end_effector.EndEffector(name, data), None


def service(name, data, child):
    if name is None or name == "":
        name = "service"
    if data is None:
        return False, None, "No data"
    if rosservice.get_service_type("/" + data['service_name']) is None:
        return False, None, "Service not found"
    return True, behaviour.service.Service(name, data), None


def navigate(name, data, child):
    if config.mode != "rolling":
        return False, None, "Need rolling mode"
    if name is None or name == "":
        name = "navigate"
    if data is None:
        return False, None, "No data"
    try:
        state, point = cartesian_point.CartesianPoint().find_db(int(data['identifier']))
    except ValueError:
        return False, None, "Point " + data + " not found"
    if not state:
        return False, None, "Point " + data + " not found"
    data = {'point': point, 'timeout': data['timeout']}
    return True, behaviour.navigate.Navigate(name, data), None


def coverage(name, data, child):
    if config.mode != "rolling":
        return False, None, "Need rolling mode"
    if name is None or name == "":
        name = "coverage"
    if data is None:
        return False, None, "No data"
    try:
        float(data['robot_width'])
    except ValueError:
        return False, None, "robot width not a float"
    if data['points'] != '':
        data['points'] = ast.literal_eval(data['points'])
        if len(Counter(data['points']).values()) < 3:
            return False, None, "No enough points"
        points = []
        for i in range(len(data['points'])):
            state, point = cartesian_point.CartesianPoint().find_db(int(data['points'][i]))
            if not state:
                return False, None, "Point " + data + " not found"
            points.append(point)
        data['points'] = points
    return True, behaviour.coverage.Coverage(name, data), None


def sleep(name, data, child):
    if name is None or name == "":
        name = "sleep"
    if data is None:
        return False, None, "No data"
    return True, behaviour.sleep.Sleep(name, data), None


def patrol(name, data, child):
    if config.mode != "rolling":
        return False, None, "Need rolling mode"
    if name is None or name == "":
        name = "patrol"
    if data is None:
        return False, None, "No data"
    if data != '':
        data = ast.literal_eval(data)
        if len(Counter(data).values()) < 2:
            return False, None, "No enough points"
        points = []
        for i in range(len(data)):
            state, point = cartesian_point.CartesianPoint().find_db(int(data[i]))
            if not state:
                return False, None, "Point " + data + " not found"
            points.append(point)
        data = points
    return True, behaviour.patrol.Patrol(name, data), None


types = {'selector': selector,
         'sequence': sequence,
         'parallel': parallel,
         'execute': execute,
         'replay': replay,
         'plan': plan,
         'cartesian': cartesian,
         'condition': condition,
         'inverter': inverter,
         'timeout': timeout,
         'record': record,
         'end effector': end_effector,
         'service': service,
         'navigate': navigate,
         'coverage': coverage,
         'sleep': sleep,
         'patrol': patrol
         }

composites = {'selector', 'sequence', 'parallel'}
leaf = {'execute', 'plan', 'cartesian', 'record', 'replay', 'end effector', 'service', 'navigate', 'coverage', 'sleep',
        'patrol'}
decorators = {'condition', 'inverter', 'timeout'}
data_node = {'execute', 'replay', 'plan', 'cartesian', 'condition', 'timeout', 'record', 'end effector', 'service',
             'navigate', 'coverage', 'sleep', 'patrol'}
states = {"success": py_trees.common.Status.SUCCESS, "failure": py_trees.common.Status.FAILURE,
          "running": py_trees.common.Status.RUNNING}
