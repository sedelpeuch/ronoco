"""
This file implements some constants for behavior like kind of types, leaves and create an instance of CartesianPoint()
commander
"""
import py_trees
from flaskr import behavior
from flaskr import cartesian_point


def selector(name, data):
    return True, py_trees.composites.Selector()


def sequence(name, data):
    return True, py_trees.composites.Sequence()


def parallel(name, data):
    return True, py_trees.composites.Parallel()


def execute(name, data):
    if data is None:
        return False, None
    state, point = cartesian_point.CartesianPoint().find_db(int(data))
    if not state:
        return False, None
    return True, behavior.execute.Execute(name, point)


def plan(name, data):
    if data is None:
        return False, None
    state, point = cartesian_point.CartesianPoint().find_db(int(data))
    if not state:
        return False, None
    return True, behavior.plan.Plan(name, point)


types = {'selector': selector,
         'sequence': sequence,
         'parallel': parallel,
         'execute': execute,
         'plan': plan,
         }

composites = {'selector', 'sequence', 'parallel'}
leaf = {'execute', 'plan'}
decorators = {'condition', 'eternalGuard', 'inverter', 'timeout'}
states = {"sucess": py_trees.Status.SUCCESS, "failure": py_trees.Status.FAILURE, "running": py_trees.Status.RUNNING}
commander = cartesian_point.CartesianPoint().commander
