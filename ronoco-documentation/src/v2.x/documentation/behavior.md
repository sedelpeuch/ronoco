# Ronoco-VM API - Behaviour documentation

The Behaviour module is the implementation in Ronoco-vm of the different blocks accessible from Node-RED. Indeed, each block on Node-RED (plan, execute, cartesian etc) is associated with a treatment in Ronoco-vm. The beahviour module is the heart of this processing and allows to identify the existing blocks and to associate functions to them.

##  <class> Behaviour </class>
List the existing blocks and their type (control node, execution nodes etc).

Defines the functions allowing to instantiate the different blocks while checking the content of the parameters.

The interface implemented forces all these functions to have name, data and child parameters.

<member> Parameters in ronoco-vm</member>

- *name*: The name of the block (optional)
- *data*: The data used to build the block (optional for some blocks, mandatory for others)
- *child*: the child of the block to instantiate in the behaviour tree (mandatory for decorators)

### <class> Common </class>
<method> <img src="../static/nodered/question-mark.svg" width="5%"></img> selector </method>

Instantiates a [selector](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#selector) node

A selector executes each of its child behaviours in turn until one of them succeeds (at which point it itself returns RUNNING or SUCCESS, or it runs out of children at which point it itself returns FAILURE. We usually refer to selecting children as a means of choosing between priorities. Each child and its subtree represent a decreasingly lower priority path.

<method> <img src="../static/nodered/right-arrow.svg" width="5%"></img> sequence </method>

Instantiates a [sequence](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#sequence) node

A sequence will progressively tick over each of its children so long as each child returns SUCCESS. If any child returns FAILURE or RUNNING the sequence will halt and the parent will adopt the result of this child. If it reaches the last child, it returns with that result regardless.

The sequence halts once it sees a child is RUNNING and then returns the result. It does not get stuck in the running behaviour.

<method> <img src="../static/nodered/parallel.svg" width="5%"></img> parallel </method>

Instantiates a [parallel](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#parallel) node

Ticks every child every time the parallel is run (a poor man’s form of paralellism).

- Parallels will return FAILURE if any child returns FAILURE
- Parallels with policy SUCCESS_ON_ONE return SUCCESS if at least one child returns SUCCESS and others are RUNNING.
- Parallels with policy SUCCESS_ON_ALL only returns SUCCESS if all children return SUCCESS

<method> <img src="../static/nodered/condition.svg" width="5%"></img> condition </method>

Instantiates a [condition](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Condition) node

Encapsulates a behaviour and wait for its status to flip to the desired state. This behaviour will tick with RUNNING while waiting and SUCCESS when the flip occurs.

<member> Parameters </member>

- *data*: SUCCESS, FAILURE or RUNNING
- *child*: must be provided

<method> <img src="../static/nodered/inverter.svg" width="5%"></img> inverter </method>

Instantiates a [inverter](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Inverter) node

A decorator that inverts the result of a class’s update function.

<member> Parameters </member>

- *child*: must be provided

<method> <img src="../static/nodered/clock.svg" width="5%"></img> timeout </method>

Instantiates a [timeout](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Timeout) node

A decorator that applies a timeout pattern to an existing behaviour. If the timeout is reached, the encapsulated behaviour’s stop() method is called with status FAILURE otherwise it will simply directly tick and return with the same status as that of it’s encapsulated behaviour.

<member> Parameters </member>

- *data*: time in seconds as an integer
- *child*: must be provided

<method> <img src="../static/nodered/service.svg" width="5%"></img> service </method>
    Allows you to call a service offered by the robot. Perform `rosservice list` to get the list of available services

<member> Parameters </member>
- *name*: String - name of current block
- *name of service*: String - The name of the service to be called. The name must not be preceded by a /.
- *Parameters of service*: Array - The parameters to be provided to the service. The parameters must be enclosed in brackets and separated by commas.

### <class> Manipulator </class>

<method> <img src="../static/nodered/play.svg" width="5%"></img> execute </method>

Execute the movement between the current position and a position given in the constructor parameter.

Use MoveGroupCommander.set_pose_target() and MoveGroupCommander.go()

<member> Parameters </member>

- *data*: identifier of a point as an integer

<method> <img src="../static/nodered/navigation.svg" width="5%"></img> plan </method>

Planning movement between the current position and a position given in the constructor parameter.

Use MoveGroupCommander.set_pose_target() and MoveGroupCommander.plan()

<member> Parameters </member>

- *data*: identifier of a point as an integer

<method> <img src="../static/nodered/cartesian.svg" width="5%"></img> cartesian </method>

Cartesian movement between the current position and a position given in the constructor parameter

Use geometry_msgs.msg.PoseStamped(), MoveGroupCommander.compute_cartesian_path() and MoveGroupCommander.execute()

<member> Parameters </member>

- *data*: a dictionary like {'point_id' : integer, 'reliability': integer (0-100), 'eef' : float}

<method> <img src="../static/nodered/record.svg" width="5%"></img> record </method>

Record all trajectories during a time specified by a parameter in constructor

<member> Parameters </member>

- *data*: a dictionary like {'identifiant' : string, 'time': integer}

<method> <img src="../static/nodered/replay.svg" width="5%"></img> replay </method>

Replays a previously recorded path

<member> Parameters </member>

- *data*: name of a recorded trajectory as a string



<method> <img src="../static/nodered/robotic-hand.svg" width="5%"></img> end_effector </method>
        Allows control of the robot end effector via an associated service passed as a parameter to ronoco.launch (the robot end effector controller must be started independently).

 This block returns SUCCESS if it has succeeded in completing the action and FAILURE if it has caught something in its path. Otherwise, the combination of "end_effector" and an "inverter" will return SUCCESS if the robot has caught something.

<member> Parameters </member>
- *name*: String - name of current block
- *data*: Array - The parameters to be provided to the service. The parameters must be enclosed in brackets and separated by commas.

### <class> Rolling </class>

<method> <img src="../static/nodered/compass.svg" width="5%"></img> navigate </method>
   Allows you to navigate to a point on a map while avoiding map obstacles.

<member> Parameters </member>
- *name*: String - Name of current block
- *data*: Integer - The identifier of the point in the ros parameter server (on the name "cartesianPoints"). Use Ronoco-ui to find out which points are registered.