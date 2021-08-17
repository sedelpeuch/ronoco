"""
This file implements the endpoint control allowing, to transform a json representing a behaviour tree into a behaviour
tree as a py_tree object then to execute it
"""

import py_trees
from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

import behaviour
import config
import logger


class Control:
    """
    Definition of the endpoint control to analyse and execute behaviour trees
    """

    def __init__(self):
        self.bp = Blueprint('control_endpoint', __name__, url_prefix='/control')

        self.bp.route('/', methods=['POST'])(self.compute)
        self.bp.route('/stop', methods=['GET'])(self.stop)
        self.behavior_tree = None
        self.behavior_tree_dict = {}
        self.trees = None
        self.roots = None

    def compute(self):
        """
        POST Method

        ROUTE /compute/

        POST body : an export of nodered tree

        This method is the main method of the endpoint to transform the json from nodered (or any other tool respecting
        the json syntax) into a behaviour tree with the py_tree package. Once built, the method call play then executes
        the different trees.

        Every behaviour tree starts with a root block, blocks that are not connected (directly or indirectly) to a root
        are ignored.

        Each root must be connected to strictly one block to be interpreted.

        To be interpreted a flow must contain at least one behaviour tree

        :return:
            - {"Success" : "All behaviour trees has been executed", 200} if the tree is built and executed correctly.
            - {"Error": "Block (or child of this block) with id <str> is incorrect", 400} if it is not
        """
        trees = []
        py_trees.logging.level = py_trees.logging.Level.DEBUG  # For development purpose only
        if request.method == 'POST':
            if config.ronoco_mode == "manipulator" and config.commander is None:
                return {"Error": "Can't connect to commander please retry with connect button"}, 404
            data = request.get_json()
            try:
                bt = data['behavior-tree']
            except KeyError:
                raise BadRequest()
            except TypeError:
                raise BadRequest()
            if bt is None or bt == {}:
                return {"Error": "json is empty"}, 400

            # find block with id "root" in json
            self.roots = self.find_roots(bt)
            if not self.roots:
                return {"Error": "json contains 0 valid roots"}, 400
            state, result = self.build_nodes(bt)
            if not state:
                return {"Error": "Block (or child of this block) with id " + result + " is incorrect"}, 400
            for node in bt:
                self.build_decorator(node, bt)
            # for each root in json build first root in py_tree and store it in trees
            self.roots.sort(key=self.get_y)
            for root in self.roots:
                if len(root['wires'][0]) != 1:
                    return {"Error": "Tree with root id " + root['id'] + " is incorrect"}, 400
                else:
                    state, root_json = self.find_by_id(root['wires'][0][0], bt)
                    if not state:
                        return {"Error": "Tree with root id " + root['id'] + " is incorrect"}, 400
                    trees.append(root_json)

            # for each json tree build it and check if it is good
            for tree in trees:
                state, result = self.build_tree(tree, bt)
                if not state:
                    return {"Error": "Block (or child of this block) with id " + result + " is incorrect"}, 400
            self.trees = trees
            return self.play()

    def play(self):
        """
        Browse the list of previously constructed trees and execute them using the tick_tock method
        :return: Message with code 200 if all trees has been executed, code 409 else
        """
        Status = py_trees.common.Status
        # for each tree execute it with tick() method
        for i in range(len(self.trees)):
            logger.info("Starting execution of tree " + self.roots[i]['name'])
            # py_trees.display.render_dot_tree(self.behavior_tree_dict[self.trees[i]['id']])
            self.behavior_tree = py_trees.trees.BehaviourTree(root=self.behavior_tree_dict[self.trees[i]['id']])
            self.behavior_tree.setup(15)
            times = None
            if self.roots[i]['data'] == "once":
                times = 1
            elif self.roots[i]['data'] == "infinite":
                times = -1
            elif self.roots[i]['data'] == "number":
                times = int(self.roots[i]['number'])
            try:
                self.behavior_tree.tick_tock(50, times)
            except KeyboardInterrupt:
                self.behavior_tree.interrupt()
            if self.behavior_tree.tip() is None or self.behavior_tree.tip().status == Status.FAILURE:
                return {"Error": "Tree with root's name :" + self.roots[i]['name'] + " can't be executed"}, 409
        return {"Success": "All behaviour trees has been executed"}, 200

    @staticmethod
    def find_roots(bt):
        """
        Search in BT-json blocks with type 'root'
        :param bt: a json representing a behaviour tree (export of nodered)
        :return: the list of roots found in the json passed in parameter if there are any, an empty list otherwise
        """
        roots = list()
        for node in bt:
            try:
                if node['type'] == 'root':
                    if node['data'] == "number":
                        try:
                            int(node['number'])
                        except KeyError:
                            return list()
                        except ValueError:
                            return list()
                    roots.append(node)
            except TypeError:
                return list()
        return roots

    @staticmethod
    def find_by_id(identifier, bt):
        """
        Search in the json passed in parameter for a block with the string id passed in parameter
        :param identifier: the id of the block searched for
        :param bt: a json representing a behaviour tree (export of nodered)
        :returns: True and the root (json) if a root has this id. False, None else
        """
        for node in bt:
            if node['id'] == identifier:
                return True, node
        return False, None

    @staticmethod
    def get_y(elem):
        """
        Return y position of an element
        """
        try:
            return elem['y']
        except KeyError:
            pass

    def build_tree(self, json_node, bt):
        """
        Builds the behaviour tree as a py_tree object from its root using a bfs algorithm
        :param json_node: a node of a tree in json format
        :param bt: a json representing a behaviour tree (export of nodered)
        :return: True and None if tree has been built. False and the id of the last block built if it could not be built
        """
        children_id = json_node['wires']
        py_tree_node = self.behavior_tree_dict[json_node['id']]

        # Check children of current node
        if not children_id:
            if json_node['type'] in behaviour.behaviour.composites:
                return False, children_id
            elif json_node['type'] in behaviour.behaviour.leaf:
                return True, None
            else:
                return False, children_id

        children = list()
        # for each children id, add it in list of node to build
        for identifier in children_id[0]:
            state, child = self.find_by_id(identifier, bt)
            if state:
                children.append(child)
            else:
                return False, identifier

        # Sort the children according to their ascending y-position. This ensures that children run from top to bottom
        children.sort(key=self.get_y)

        # For each child verify if it's a decorator then connect child with his father
        for child in children:
            if child['type'] in behaviour.behaviour.decorators:
                if len(child['wires'][0]) != 1:
                    return False, child['id']
                grandson = self.behavior_tree_dict[child['wires'][0][0]]
                name = None
                data = None
                try:
                    name = child['name']
                    data = child['data']
                except KeyError:
                    pass
                state, node_py_tree = behaviour.behaviour.types[child['type']](name=name, data=data, child=grandson)
                if not state:
                    return False, child['id']
                self.behavior_tree_dict[child['id']] = node_py_tree
            try:
                py_tree_node.add_child(self.behavior_tree_dict[child['id']])
            except AttributeError:
                pass
            self.build_tree(child, bt)
        return True, None

    def build_decorator(self, node, bt):
        """
        Transforms a decorator from a json to a py_tree object. In order to build the decorators it is necessary
        that the other blocks are already built
        :param node: the decorator in json format
        :param bt: a json representing a behaviour tree (export of nodered)
        :return: True, None if everything is ok, False and an id else
        """
        if node['type'] in behaviour.behaviour.decorators:
            state, child = self.find_by_id(node['wires'][0][0], bt)
            if child['type'] in behaviour.behaviour.decorators:
                self.build_decorator(child, bt)
            else:
                name = None
                data = None
                if node['name'] != "":
                    name = node['name']
                try:
                    data = node['data']
                except KeyError:
                    pass
                child = self.behavior_tree_dict[child['id']]
                state, node_py_tree = behaviour.behaviour.types[node['type']](name=name, data=data, child=child)
                if not state:
                    return False, child['id']
                self.behavior_tree_dict[node['id']] = node_py_tree

    @staticmethod
    def multiple_data_nodes(node_json):
        """
        Analyses the data of a node and according to its type correctly formats the data dictionary.

        Normally this function only returns the data field of the node but some special nodes escape this rule.

        :param node_json: the node to evaluate
        :return: False, None if a data is expected but not present. True and the correctly formatted dictionary else
        """
        data = {}
        if node_json['type'] in behaviour.behaviour.data_node:
            try:
                if node_json['type'] == 'cartesian':
                    data = {'point_id': node_json['point_id'], 'reliability': node_json['reliability'],
                            'eef': node_json['eef']}
                elif node_json['type'] == 'record':
                    data = {'identifier': node_json['identifier'], 'time': node_json['time']}
                elif node_json['type'] == 'service':
                    data = {'service_name': node_json['service_name'],
                            'service_parameter': node_json['service_parameter']}
                elif node_json['type'] == 'navigate':
                    data = {'identifier': node_json['data'], 'timeout': node_json['timeout']}
                else:
                    data = node_json['data']
            except KeyError:
                return False, None
        return True, data

    def build_nodes(self, bt):
        """
        Transforms all the nodes of the tree from json form to py_tree form. This function does not handle decorator
        blocks that need special treatment concerning their children
        :param bt: a json representing a behaviour tree (export of nodered)
        :return: True, None if everything is ok, False and an id else
        """
        self.behavior_tree_dict = {}
        for node_json in bt:
            name = None
            data = None
            try:
                name = node_json['name']
                state, data = self.multiple_data_nodes(node_json)
                if not state:
                    return False, node_json['id']
            except KeyError:
                pass
            if node_json['type'] != "tab" and node_json['type'] != "root" \
                    and node_json['type'] not in behaviour.behaviour.decorators:
                try:
                    state, node_py_tree = behaviour.behaviour.types[node_json['type']](name=name, data=data, child=None)
                except KeyError:
                    return False, node_json['id']
                if not state:
                    return False, node_json['id']
                self.behavior_tree_dict[node_json['id']] = node_py_tree
        return True, None

    def stop(self):
        """
        Stop execute of current behaviour tree
        """
        self.behavior_tree.interrupt()
        return {"Success": "Behavior tree has been stopped "}, 200
