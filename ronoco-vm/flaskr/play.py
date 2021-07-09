"""
This file implements the endpoint play allowing, to transform a json representing a behaviour tree into a behaviour tree
as a py_tree object then to execute it
"""

import py_trees
from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

from flaskr import behavior


class Play:
    """
    Definition of the endpoint play to analyse and execute behaviour trees
    """

    def __init__(self):
        self.bp = Blueprint('play_endpoint', __name__, url_prefix='/play')

        self.bp.route('/', methods=['POST'])(self.play)
        self.bp.route('/stop', methods=['GET'])(self.stop)
        self.behavior_tree = None
        self.behavior_tree_dict = {}

    def play(self):
        """
        POST Method

        ROUTE /play/

        POST body : an export of nodered tree

        This method is the main method of the endpoint to transform the json from nodered (or any other tool respecting
        the json syntax) into a behaviour tree with the py_tree package. Once built, the method executes the different
        trees.

        Every behaviour tree starts with a root block, blocks that are not connected (directly or indirectly) to a root
        are ignored.

        Each root must be connected to strictly one block to be interpreted.

        To be interpreted a flow must contain at least one behaviour tree

        :return:
            - {"Success" : "All behavior trees has been executed", 200} if the tree is built and executed correctly.
            - {"Error": "Block (or child of this block) with id <str> is incorrect", 400} if it is not
        """
        trees = []
        py_trees.logging.level = py_trees.logging.Level.DEBUG  # For development purpose only

        if request.method == 'POST':
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
            roots = self.find_roots(bt)
            if not roots:
                return {"Error": "json contains 0 valid roots"}, 400

            state, result = self.build_nodes(bt)
            for node in bt:
                self.build_decorator(node, bt)
            if not state:
                return {"Error": "Block (or child of this block) with id " + result + " is incorrect"}, 400

            # for each root in json build first root in py_tree and store it in trees
            for root in roots:
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

            # for each tree execute it with tick() method
            for tree in trees:
                py_trees.display.render_dot_tree(self.behavior_tree_dict[tree['id']])
                self.behavior_tree = py_trees.trees.BehaviourTree(root=self.behavior_tree_dict[tree['id']])
                self.behavior_tree.setup(15)
                try:
                    self.behavior_tree.tick()
                except KeyboardInterrupt:
                    self.behavior_tree.interrupt()
            return {"Success": "All behavior trees has been executed"}, 200

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
                    roots.append(node)
            except TypeError:
                return list()
        return roots

    @staticmethod
    def find_by_id(identifiant, bt):
        """
        Search in the json passed in parameter for a block with the string id passed in parameter
        :param identifiant: the id of the block searched for
        :param bt: a json representing a behaviour tree (export of nodered)
        :returns: True and the root (json) if a root has this id. False, None else
        """
        for node in bt:
            if node['id'] == identifiant:
                return True, node
        return False, None

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
            if json_node['type'] in behavior.behavior.composites:
                return False, children_id
            elif json_node['type'] in behavior.behavior.leaf:
                return True, None
            else:
                return False, children_id

        children = list()
        # for each children id, add it in list of node to build
        for identifiant in children_id[0]:
            state, child = self.find_by_id(identifiant, bt)
            if state:
                children.append(child)
            else:
                return False, identifiant

        def get_y(elem):
            """
            Return y position of an element
            """
            try:
                return elem['y']
            except KeyError:
                pass

        # Sort the children according to their ascending y-position. This ensures that children run from top to bottom
        children.sort(key=get_y)

        # For each child verify if it's a decorator then connect child with his father
        for child in children:
            if child['type'] in behavior.behavior.decorators:
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
                state, node_py_tree = behavior.behavior.types[child['type']](name=name, data=data, child=grandson)
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
        if node['type'] in behavior.behavior.decorators:
            state, child = self.find_by_id(node['wires'][0][0], bt)
            if child['type'] in behavior.behavior.decorators:
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
                state, node_py_tree = behavior.behavior.types[node['type']](name=name, data=data, child=child)
                if not state:
                    return False, child['id']
                self.behavior_tree_dict[node['id']] = node_py_tree

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
                data = node_json['data']
            except KeyError:
                pass
            if node_json['type'] != "tab" and node_json['type'] != "root" and node_json[
                'type'] not in behavior.behavior.decorators:
                state, node_py_tree = behavior.behavior.types[node_json['type']](name=name, data=data, child=None)
                if not state:
                    return False, node_json['id']
                self.behavior_tree_dict[node_json['id']] = node_py_tree
        return True, None

    def stop(self):
        """
        Stop execute of current behavior tree
        """
        self.behavior_tree.interrupt()
        return {"Success": "Behavior tree has been stopped "}, 200
