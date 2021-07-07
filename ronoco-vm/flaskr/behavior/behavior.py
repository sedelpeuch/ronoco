"""
This file implements some constants for behavior like kind of types, leaves and create an instance of CartesianPoint()
commander
"""
import py_trees
from flaskr import behavior
from flaskr import cartesian_point


def selector(name, data, child):
    return True, py_trees.composites.Selector()


def sequence(name, data, child):
    return True, py_trees.composites.Sequence()


def parallel(name, data, child):
    return True, py_trees.composites.Parallel()


def execute(name, data, child):
    if data is None:
        return False, None
    state, point = cartesian_point.CartesianPoint().find_db(int(data))
    if not state:
        return False, None
    return True, behavior.execute.Execute(name, point)


def plan(name, data, child):
    if data is None:
        return False, None
    state, point = cartesian_point.CartesianPoint().find_db(int(data))
    if not state:
        return False, None
    return True, behavior.plan.Plan(name, point)


def condition(name, data, child):
    if data is None:
        return False, None
    try:
        return True, py_trees.decorators.Condition(name=name, status=states[data], child=child)
    except KeyError:
        return False, None


# def eternal_guard(name, data, child):
#     if data is None:
#         return False, None
#     try:
#         return True, py_trees.decorators.EternalGuard(name=name, status=states[data], child=child)
#     except KeyError:
#         return False, None

def inverter(name, data, child):
    return True, py_trees.decorators.Inverter(name=name, child=child)


def timeout(name, data, child):
    if data is None:
        return False, None
    try:
        return True, py_trees.decorators.Timeout(name=name, duration=int(data), child=child)
    except TypeError:
        return False, None


types = {'selector': selector,
         'sequence': sequence,
         'parallel': parallel,
         'execute': execute,
         'plan': plan,
         'condition': condition,
         # 'eternalGuard': eternal_guard,
         'inverter': inverter,
         'timeout': timeout
         }

composites = {'selector', 'sequence', 'parallel'}
leaf = {'execute', 'plan'}
decorators = {'condition', 'eternalGuard', 'inverter', 'timeout'}
states = {"success": py_trees.Status.SUCCESS, "failure": py_trees.Status.FAILURE, "running": py_trees.Status.RUNNING}
commander = cartesian_point.CartesianPoint().commander
