# Ronoco-vm API - Detailed documentation

<!-- toc -->

##  <class> Common </class>
Definition of common endpoint

<method> index() </method>

GET Method

ROUTE /

<member>Return</member> : if everything is ok : {"Success": "Server is running"}, 200

<method> send_states()</method>
This function check if the state of the robot or rviz has changed. If so, it sends a message to the websockets states
channel

<method> robot_state()</method>

Check if you can communicate with a controller
- Use rosservice /rosout/get_loggers
- Node: /rosout
- Type: roscpp/GetLoggers
- Args:

<member>Return</member> True if communication with rosmaster is possible, False else

<method> rviz_state() </method>

Check if rviz send data on topic /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
/update

<member>Return</member> False if rviz doesn't send data, True else

<method> shutdown() </method>

GET Method

ROUTE /shutdown

Shutdown server with config.socketio.stop() (and shutdown send_states() daemon)

## <class> Free </class>
Implements free endpoints to set and get the compliance (or 0 gravity robot WIP) of robot

<method> free() </method>

GET/POST Method

ROUTE /free/

POST body
{
"compliant" : "True"/"False"
}

Allows you to get or set robot compliance
- Use rosservice /set_compliant
- Node: /joint_trajectory_action_server
- Type: std_srvs/SetBool
- Args: data

<member> Return </member> if everything is ok : {"compliant":"True"/"False"} else an HttpError

## <class> CartesianPoint </class>
Allows to add, get and delete an cartesian point in the ros parameters server from Rviz or compliant mode

<method> add_bd(cartesian_point) </method>

This method adds a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method is called
when a POST request is made on point/add/simulation or point/add/real

<member>Parameters </member>
- *cartesian_point:* a cartesian point eg {"position": {"x": pose.x, "y": pose.y, "z": pose.z}, "orientation": {"x":
  orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}}

<member> Return </member> True if everything is ok

<method> delete_db(identifiant) </method>

This method delete a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method
is called when a DELETE request is made on point/delete

<member>Parameters </member>
- *identifiant:* a number

<member> Return </member> True if point is deleted, False if point doesn't exist

<method> find_db(identifiant) </method>

This method find a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method is called
when a GET request is made on point/get/number or point/get

<member>Parameters </member>
- *identifiant:* a number

<member>Return</member> True if point is found, False if point doesn't exist

<method> clear_db() </method>

This method delete all cartesian points in the ros parameters server (on the name "cartesianPoints"). This
method is called when a POST request is made on point/delete

<method> add_point_from_actual_position() </method>

POST Method

ROUTE /point/add/actual

POST body
{
}

Allows you to add a Cartesian point corresponding to the current position of the robot.

The id of the position is automatically given

<member> Return </member> id for new cartesian point if everything is ok, a 409 error else

<method> add_point_from_rviz() </method>

POST Method

ROUTE /point/add/simulation

POST body
{
}

Allows you to add a Cartesian point corresponding to the current position of the robot in Rviz.

The id of the position is automatically given

<member> Return </member> id for new cartesian point if everything is ok, a 408 error else

<method> get_all_points() </methods>


GET Method

ROUTE /point/get

Allows you to get all Cartesian points in the ros parameters server (on the name "cartesianPoints")

<member> Return </member> a json with cartesian points if everything is ok, a 404 error else

<method> get_one_point(identifiant) </method>

GET Method

ROUTE /point/get/<identifiant>

Allows you to get one Cartesian point in the ros parameters server (on the name "cartesianPoints")

<member>Parameters </member>
- *identifiant:* a number

<member> Return </member> a json with cartesian point it exists, a 404 error else

<method> delete_one_point(identifiant) </method>

POST Method

ROUTE /point/delete/<identifiant>

POST body
{
}

Allows you to delete one Cartesian point in the ros parameters server (on the name "cartesianPoints")

<member>Parameters </member>
- *identifiant:* a number

<member> Return </member> a response 200 if the point have been deleted, a 404 error else

<method> delete_all_points </method>

POST Method

ROUTE /point/delete

POST body
{
}

Allows you to delete all Cartesian points in the ros parameters server (on the name "cartesianPoints")

<member> Return </member> a response 200

## <class> Control </class>

Definition of the endpoint control to analyse and execute behaviour trees

<method> compute() </method>

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

<member> Return </member>
- {"Success" : "All behavior trees has been executed", 200} if the tree is built and executed
correctly.
- {"Error": "Block (or child of this block) with id <str> is incorrect", 400} if it is not

<method> play() </method>

Browse the list of previously constructed trees and execute them using the tick_tock method
<member> Return </member> Message with code 200 if all trees has been executed, code 409 else

<method> find_roots(bt) </method>

Search in BT-json blocks with type 'root'

<member>Parameters </member>
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> the list of roots found in
  the json passed in parameter if there are any, an empty list otherwise

<method> find_by_id(identifiant, bt) </method>
Search in the json passed in parameter for a block with the string id passed in parameter

<member>Parameters </member>
- *identifiant:* the id of the block searched for
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> True and the root (json) if a root has this id. False, None else

<method> build_tree(json_node, bt) </method>

Builds the behaviour tree as a py_tree object from its root using a bfs algorithm

<member>Parameters </member>
- *json_node:* a node of a tree in json format
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> True and None if tree has been built. False
  and the id of the last block built if it could not be built

<method> build_decorator(node, bt) </method>

Transforms a decorator from a json to a py_tree object. In order to build the decorators it is necessary
that the other blocks are already built

<member>Parameters </member>
- *node:* the decorator in json format
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> True, None if everything is ok, False and an id else

<method> build_nodes(bt) </method>

Transforms all the nodes of the tree from json form to py_tree form. This function does not handle decorator
blocks that need special treatment concerning their children

<member>Parameters </member>
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> True, None if everything is ok, False and an id else

<method> stop() </method>

Stop execute of current behavior tree

