# Ronoco-VM API - Behaviour documentation

The Behaviour module is the implementation in Ronoco-vm of the different blocks accessible from Node-RED. Indeed, each block on Node-RED (plan, execute, cartesian etc) is associated with a treatment in Ronoco-vm. The beahviour module is the heart of this processing and allows to identify the existing blocks and to associate functions to them.

##  <class> Behaviour </class>
List the existing blocks and their type (control node, execution nodes etc).

Defines the functions allowing to instantiate the different blocks while checking the content of the parameters.

The interface implemented forces all these functions to have name, data and child parameters.

<member> Parameters </member>

- *name*: The name of the block (optional)
- *data*: The data used to build the block (optional for some blocks, mandatory for others)
- *child*: the child of the block to instantiate in the behaviour tree (mandatory for decorators)

<method> selector(name, data, child) </method>

Instantiates a [selector](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#selector) node

A selector executes each of its child behaviours in turn until one of them succeeds (at which point it itself returns RUNNING or SUCCESS, or it runs out of children at which point it itself returns FAILURE. We usually refer to selecting children as a means of choosing between priorities. Each child and its subtree represent a decreasingly lower priority path.

<method> sequence(name, data, child) </method>

Instantiates a [sequence](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#sequence) node

A sequence will progressively tick over each of its children so long as each child returns SUCCESS. If any child returns FAILURE or RUNNING the sequence will halt and the parent will adopt the result of this child. If it reaches the last child, it returns with that result regardless.

The sequence halts once it sees a child is RUNNING and then returns the result. It does not get stuck in the running behaviour.

<method> parallel(name, data, child) </method>

Instantiates a [parallel](https://py-trees.readthedocs.io/en/release-0.7.x/composites.html#parallel) node

Ticks every child every time the parallel is run (a poor man’s form of paralellism).

- Parallels will return FAILURE if any child returns FAILURE
- Parallels with policy SUCCESS_ON_ONE return SUCCESS if at least one child returns SUCCESS and others are RUNNING.
- Parallels with policy SUCCESS_ON_ALL only returns SUCCESS if all children return SUCCESS

<method> condition(name, data, child) </method>

Instantiates a [condition](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Condition) node

Encapsulates a behaviour and wait for its status to flip to the desired state. This behaviour will tick with RUNNING while waiting and SUCCESS when the flip occurs.

<member> Parameters </member>

- *data*: SUCCESS, FAILURE or RUNNING
- *child*: must be provided

<method> inverter(name, data, child) </method>

Instantiates a [inverter](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Inverter) node

A decorator that inverts the result of a class’s update function.

<member> Parameters </member>

- *child*: must be provided

<method> timeout(name, data, child) </method>

Instantiates a [timeout](https://py-trees.readthedocs.io/en/release-0.7.x/modules.html#py_trees.decorators.Timeout) node

A decorator that applies a timeout pattern to an existing behaviour. If the timeout is reached, the encapsulated behaviour’s stop() method is called with status FAILURE otherwise it will simply directly tick and return with the same status as that of it’s encapsulated behaviour.

<member> Parameters </member>

- *data*: time in seconds as an integer
- *child*: must be provided

<method> execute(name, data, child) </method>

Execute the movement between the current position and a position given in the constructor parameter.

Use MoveGroupCommander.set_pose_target() and MoveGroupCommander.go()

<member> Parameters </member>

- *data*: identifier of a point as an integer

<method> plan(name, data, child) </method>

Planning movement between the current position and a position given in the constructor parameter.

Use MoveGroupCommander.set_pose_target() and MoveGroupCommander.plan()

<member> Parameters </member>

- *data*: identifier of a point as an integer

<method> cartesian(name, data, child) </method>

Cartesian movement between the current position and a position given in the constructor parameter

Use geometry_msgs.msg.PoseStamped(), MoveGroupCommander.compute_cartesian_path() and MoveGroupCommander.execute()

<member> Parameters </member>

- *data*: a dictionary like {'point_id' : integer, 'reliability': integer (0-100), 'eef' : float}

<method> record(name, data, child) </method>

Record all trajectories during a time specified by a parameter in constructor

<member> Parameters </member>

- *data*: a dictionary like {'identifiant' : string, 'time': integer}

<method> replay(name, data, child) </method>

Replays a previously recorded path

<member> Parameters </member>

- *data*: name of a recorded trajectory as a string