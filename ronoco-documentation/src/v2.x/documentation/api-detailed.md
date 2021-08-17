# Ronoco-vm API - Detailed documentation

- [Run](api-detailed.html#class-run-class)<!-- @IGNORE PREVIOUS: link -->
- [Common](api-detailed.html#class-common-class)<!-- @IGNORE PREVIOUS: link -->
- [Free](api-detailed.html#class-free-class)<!-- @IGNORE PREVIOUS: link -->
- [Cartesian Point](api-detailed.html#class-cartesianpoint-class)<!-- @IGNORE PREVIOUS: link -->
- [Control](api-detailed.html#class-control-class)<!-- @IGNORE PREVIOUS: link -->
- [Recorder](api-detailed.html#class-recorder-class)<!-- @IGNORE PREVIOUS: link -->
- [Logger](api-detailed.html#class-logger-class)<!-- @IGNORE PREVIOUS: link -->
- [Teleoperation](api-detailed.html#class-teleoperation-class)<!-- @IGNORE PREVIOUS: link -->

## <class> Run </class>

Define and setup flask server and rostopic subscriber / publisher for ronoco-vm

<method> create_app(test_config) </method>
Build a Flask instance and configure it

<member>Parameters </member>
- *test_config:* path to configuration file (Default : None)

<member>Return</member> a Flask instance

<method> setup_app() </method>

Register blueprint in app.

The class attribute "app" must contain a Flask instance

<member>Return</member> None

<method> subscribe_topic() </method>

Uses rospy to subscribe to the different topics needed by the API

##  <class> Common </class>
Definition of common endpoint

<method> index() </method>

GET Method

ROUTE /

<member>Return</member> if everything is OK : {"Success": "Server is running"}, 200

<method> send_states()</method>
Sends every 5 seconds the server status on the websocket "states" channel.

The server state depends on the operating mode of ronoco.

In manipulator mode the state of ros, MoveIt, rviz and the MoveGroupCommander

In rolling mode, the state of ros, rviz and the various topics necessary for the operation of rolling robots
(cmd_vel, move_base, amcl_pose)

<method> ros_state()</method>

Check if you can communicate with MoveIT
- Use rosservice /move_group/get_loggers
- Node: /move_group
- Type: roscpp/GetLoggers
- Args:

<member>Return</member> True if communication with rosmaster is possible, False else

<method> moveit_state() </method>

Check if you can communicate with moveit
- Use rosservice /move_group/get_loggers
- Node: /move_group
- Type: roscpp/GetLoggers
- Args:

<member>Return</member> True if communication with moveit is possible, False else

<method> rviz_state() </method>

Check if rviz send data on topic /rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic
/update

<member>Return</member> False if rviz doesn't send data, True else

<method> commander_state() </method>

Check if commander is initialized

<member>Return</member> False if not, True else

<method> navigation_states() </method>

Check if you can communicate with move_base and amcl_pose
+ Use rosservice /move_base/get_loggers
+ Node: /move_base
+ Type: roscpp/GetLoggers
+ Args:

+ Use rosservice /amcl_pose/get_loggers
+ Node: /amcl_pose
+ Type: roscpp/GetLoggers
+ Args:

<member>Return</member> False if not, True else

<method> shutdown() </method>

GET Method

ROUTE /shutdown

Shutdown server with config.socketio.stop() (and shutdown send_states() daemon)

<method> connect() </method>

GET Method

ROUTE /connect

Connect to config.move_group with moveit_commander.MoveGroupCommander

<member>Return</member> {"Error": "Can't connect to commander please retry with connect button"}, 404 if it's not possible,
        {"Success": "Connected with commander " + config.move_group}, 200 else

<method> shutdown() </method>

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

<member> Return </member> if everything is OK : {"compliant":"True"/"False"} else an HttpError

## <class> CartesianPoint </class>
Allows adding, get and delete a cartesian point in the ros parameters server from Rviz or compliant mode

<method> add_bd(cartesian_point) </method>

This method adds a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method is called
when a POST request is made on point/add/simulation or point/add/real

<member>Parameters </member>
- *cartesian_point:* a cartesian point eg {"position": {"x": pose.x, "y": pose.y, "z": pose.z}, "orientation": {"x":
  orientation.x, "y": orientation.y, "z": orientation.z, "w": orientation.w}}

<member> Return </member> True if everything is OK

<method> delete_db(identifiant) </method>

This method deletes a cartesian point in the ros parameters server (on the name "cartesianPoints"). This method
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

This method deletes all cartesian points in the ros parameters server (on the name "cartesianPoints"). This
method is called when a POST request is made on point/delete

<method> manipulator_add_point_real() </method>

POST Method

ROUTE /point/add/actual

POST body
{
}

Manipulator only

Allows you to add a Cartesian point corresponding to the current position of the robot.

The id of the position is automatically given

<member> Return </member> id for new cartesian point if everything is OK, a 409 error else

<method> rolling_add_point_real() </method>

POST Method

ROUTE /point/add/actual

POST body
{
}

Rolling only

Allows you to add a Cartesian point corresponding to the current position of the robot.

The id of the position is automatically given

<member> Return </member> id for new cartesian point if everything is OK, a 408 error else

<method> add_point_simulation() </method>

POST Method

ROUTE /point/add/simulation

POST body
{
}

Allows you to add a Cartesian point corresponding to the current position of the robot in Rviz.

The id of the position is automatically given

<member> Return </member> id for new cartesian point if everything is OK, a 408 error else

<method> get_all_points() </methods>


GET Method

ROUTE /point/get

Allows you to get all Cartesian points in the ros parameters server (on the name "cartesianPoints")

<member> Return </member> a json with cartesian points if everything is OK, a 404 error else

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

<member> Return </member> True, None if everything is OK, False and an id else

<method> multiple_data_nodes(node_json) </method>

Analyses the data of a node and according to its type correctly formats the data dictionary.

Normally this function only returns the data field of the node but some special nodes escape this rule.

<member>Parameters </member>
- *node_json:* the node to evaluate

<member> Return </member> False, None if a data is expected but not present. True and the correctly formatted dictionary else

<method> build_nodes(bt) </method>

Transforms all the nodes of the tree from json form to py_tree form. This function does not handle decorator
blocks that need special treatment concerning their children

<member>Parameters </member>
- *bt:* a json representing a behaviour tree (export of nodered)

<member> Return </member> True, None if everything is OK, False and an id else

<method> stop() </method>

Stop execute of current behavior tree

## <class> Recorder </class>

Recorder class copy from [Poppy project](https://github.com/poppy-project/poppy_controllers/blob/v1.0/src/poppy_ros_control/recorder.py)

<method> start_recording </method>

Start the recording of a trajectory

<member> Return </member> False if a record is already in progress, True else

<method> stop_and_save(trajectory_name) </method>

Stop recording then save it as a json file

<member>Parameters </member>

- *trajectory_name:* record's name

<member> Return </member> False if another record is in progress, True else

<method> load(, trajectory_name) </method>

Find in the trajectory directory json file with a specific name the convert it in a RobotTrajectory
<member>Parameters </member>

- trajectory_name: name of a previously recorded path

<member> Return </member> None if the trajectory does not exist, a RobotTrajectory else

## <class> Logger </class>

Definition of a logger for the websocket to be displayed in ronoco-ui

<method> logger(msg) </method>

Send a message to the /control_log namespace via socketio's emit method
The message must be a dictionary with at least one key/value

Some particular keys will be interpreted and reformatted by ronoco-ui (Debug, Success, Error, Warning, Info)

The other keys will be displayed rawly way

<member>Parameters </member>
- *msg:* the message to send

<method> debug(msg) </method>

Sends a message with the debug key if the verbosity level is greater than or equal to 4

<member>Parameters </member>
- *msg:* a string message

<method> info(msg) </method>

Sends a message with the info key if the verbosity level is greater than or equal to 3

<member>Parameters </member>
- *msg:* a string message

<method> warn(msg) </method>

Sends a message with the warn key if the verbosity level is greater than or equal to 2

<member>Parameters </member>
- *msg:* a string message

<method> error(msg) </method>

Sends a message with the error key if the verbosity level is greater than or equal to 1

<member>Parameters </member>
- *msg:* a string message

## <class> Teleoperation </class>

Definition of endpoint teleoperation allowing to manually move rolling robot

<method> set_vel(direction) </method>

POST Method

POST Body
{}

ROUTE /teleop/direction

Where direction is :
- force-stop
- forward
- backward
- left
- right

Set a twist vector with desired velocity then launch thread to execute it

<member>Parameters </member>
- *direction:* desired direction in force-stop, forward, backward, left, right

<member> Return </member> Success, 200

<method> move() </method>

Target of thread which continuously publish actualised twist message on topic cmd_vel

<method> forward() </method>

Add 0.01 to twist linear x (forward)

<method> backward() </method>

Remove 0.01 to twist linear x (backward)

<method> left() </method>

Add 0.1 to twist angular z (left)

<method> right() </method>

Remove 0.1 to twist angular z (right)

<method> force_stop() </method>

Set to 0.0 twist linear x and twist angular z (stop)