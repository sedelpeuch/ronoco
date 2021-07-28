# How to create a behaviour tree

This page is an introduction to the behaviour trees used in Ronoco to define programs. It is inspired by [Robotic Sea Bass - Introduction to Behavior Trees](https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/),
and the [documentation of py_trees](https://py-trees.readthedocs.io/en/devel/)

## What is a Behaviour Tree ?

There are several abstractions to help design complex behaviors for an autonomous agent. Generally, these consist of a finite set of entities that map to particular behaviors or operating modes within our system, e.g., “move forward”, “close gripper”, “blink the warning lights”, “go to the charging station”. Each model class has some set of rules that describe when an agent should execute each of these behaviors, and more importantly how the agent should switch between them.

Behavior trees (BTs) are one such abstraction, which I will define by the following characteristics:
1. **Behavior Trees and trees** : They start at a root node and are designed to be traversed in a specific order until a terminal state is reached
2. **Leaf nodes are executable behaviours** : Each leaf will do *something*, whether it's a simple check or a complex action, and will output a status (success, failure or running). In other words, leaf nodes are where you connect a BT to the lower-level code for your specific application.
3. **Internal nodes control tree traversal**: The internal nodes of the tree will accept the resulting status of their children and apply their own rules to dictate which node should be expanded next.

## Behaviour Tree Terminology

Let's dig into the terminology in behaviour trees.

At a glance, these are the types of nodes that make up behaviour trees and how they are represented graphically

<center>
<img src="terminology.png" width=100%></img>
</center>

Behaviour trees execute in discrete update steps known as **ticks**. When a BT is ticked, usually at some specified rate, its child nodes recursively tick based on how the tree is constructed. After a node ticks, it returns a **status** to its parent, which can be **Sucess**, **Failure**, or **Running**.

**Execution nodes**, which are leaves of the BT, can either be **Action** or **Condition** nodes. The only diffrence is that nodes can only return Success or Failure within a single tick, whereas action nodes can span multiple ticks and can return Running until they reach a terminal state. Generally, condition nodes represent simple checks (e.g., "is the gripper open ?") while action nodes represent complex actions (e.g. "open the door").

**Control nodes** are internal nodes and define how to traverse the BT given the status of their children. Importantly children of control nodes can be execution nodes or control nodes themselves. **Sequence**, **Selector** and **Parallel** nodes can have any number of children, but differ in how they process said children. **Decorator** nodes necessarily have one child, and modify its behavior with some custom defined policy.

- **Sequence** nodes execute children in order until one child returns Failure or all children returns Success
- **Selector** nodes execute children in order until one of them returns Success or all children return Failure.
- **Parallel** nodes will execute all their children in "parallel". This is in quotes because it's not true parallelism, at each tick, each node will individually tick in order. Parallel nodes return Success when at least one child nodes have succeeded, and Failure when all child nodes have failed.
- **Decorator** nodes modify a single child node with a custom policy. For example "inverter" decorator will change Success to Failure and vice-versa. To see all available decorators refer to the [Ronoco Node-RED documentation](../documentation/nodered.md).

## Examples

In Node-RED the behaviour trees are read from left to right and from top to bottom. In the first example, the "root" node is read first, then "sequence", then "plan" and finally "execute".

Let's start with a simple example, a sequence to plan a path to a point and then execute it. If the planning to the point succeeds then the execution will be done. However, if the planning fails, the sequence will be interrupted and the execution will not take place.

<center>
<img src="example1.png" width=70%></img>
</center>

In this more complete example we will want to go to the original position and then perform a Cartesian move to position 1 in less than 10 seconds otherwise go to position 2.

To do this we start with a sequence of two children. The first one asks to move to the home position, then if the execution is successful we introduce a two-child selector. The first one asks to perform a Cartesian move to position 1, it is accompanied by a timeout decorator set to 10 seconds. If the timeout is exceeded or the move is not feasible then this first child returns a failure. At this point the selector will execute its second child consisting of moving to position 2. If the first child returns a success the selector does not need to execute its second child and the execution of this tree ends.

<center>
<img src="example2.png" width=100%></img>
</center>
