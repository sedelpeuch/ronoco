"""
This file implements the endpoint play allowing, to transform a json representing a behaviour tree into a behaviour tree
as a py_tree object then to execute it
"""

from flask import Blueprint, request
from werkzeug.exceptions import BadRequest

import py_trees
from flaskr import behavior, cartesian_point


class Play:
    """
    Definition of the endpoint play to analyse and execute behaviour trees
    """

    def __init__(self):
        self.bp = Blueprint('play_endpoint', __name__, url_prefix='/play')

        self.bp.route('/', methods=['POST'])(self.play)
        self.bp.route('/stop', methods=['GET'])(self.stop)
        self.behavior_tree = None

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

            # for each root in json build first root in py_tree and store it in trees
            for root in roots:
                if len(root['wires'][0]) != 1:
                    return {"Error": "Tree with root id " + root['id'] + " is incorrect"}, 400
                else:
                    state, child = self.find_by_id(root['wires'][0][0], bt)
                    if state:
                        state, root = self.build_root(child)
                        if state:
                            trees.append((child, root))
                        else:
                            return {"Error": "Tree with root id " + root['id'] + " is incorrect"}, 400
                    else:
                        return {"Error": "Tree with root id " + root['id'] + " is incorrect"}, 400

            # for each json tree build it and check if it is good
            for tree in trees:
                state, result = self.build_tree(tree[0], tree[1], bt)
                if not state:
                    return {"Error": "Block (or child of this block) with id " + result['id'] + " is incorrect"}, 400

            # for each tree execute it with tick() method
            for tree in trees:
                py_trees.display.render_dot_tree(tree[1])
                self.behavior_tree = py_trees.trees.BehaviourTree(root=tree[1])
                self.behavior_tree.setup(timeout=15)
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

    @staticmethod
    def build_root(root):
        """
        Builds the root of a behaviour tree as a py_tree object. In other words, constructs in py_tree the only child
        of a json root block
        :param root: a json root node
        :return: True and the root (py_tree) if a root has this id. False, None else
        """
        try:
            root_node = behavior.behavior.types[root['type']]()
        except KeyError:
            return False, None
        return True, root_node

    def build_tree(self, json_node, bt_node, bt):
        """
        Builds the behaviour tree as a py_tree object from its root using a bfs algorithm
        :param json_node: a node of a tree in json format
        :param bt_node: a node of a tree in py_tree format
        :param bt: a json representing a behaviour tree (export of nodered)
        :return: True and None if tree has been built. False and the id of the last block built if it could not be built
        """
        children_id = json_node['wires']

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
        for child in children:
            name = None
            identifiant = None
            point = None
            state = True
            if child['name'] != "":
                name = child['name']
            try:
                identifiant = int(child['number'])
                # if it has an id, check if it's in the ros parameters server (on the name "cartesianPoints")
                state, point = cartesian_point.CartesianPoint().find_db(identifiant)
            except KeyError:
                pass
            except TypeError:
                return False, child
            if not state:
                return False, child
            try:
                # check behavior.py for available types
                if name is not None and identifiant is not None:
                    child_node = behavior.behavior.types[child['type']](name=name, id=identifiant, point=point)
                elif name is not None:
                    child_node = behavior.behavior.types[child['type']](name=name)
                elif identifiant is not None:
                    child_node = behavior.behavior.types[child['type']](id=identifiant, point=point)
                else:
                    child_node = behavior.behavior.types[child['type']]()
            except KeyError:
                return False, child['id']
            bt_node.add_child(child_node)
            self.build_tree(child, child_node, bt)
        return True, None

    def stop(self):
        """
        Stop execute of current behavior tree
        """
        self.behavior_tree.interrupt()
        return {"Success": "Behavior tree has been stopped "}, 200
