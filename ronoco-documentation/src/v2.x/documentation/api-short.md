# Ronoco-vm API - Summarized documentation

The table below lists all the endpoints of **ronoco-vm** and their different possible behaviour

| URL          | Method | Body | Return Code     |    Return          |      Description            |
|:-------------|:--------| :--- | :---- |:-----|:---------------------------------|
| /            | GET     | | 200  | {"Success": "Server is running"} |                        |
| /shutdown    | GET     | |      |                                  | Shutdown flask server  |
| /connect    | GET     | |  200    |   {"Success": "Connected with commander " + move_group}                               | Connexion with commander etablished  |
| /connect    | GET     | |  404    |   {"Error": "Can't connect to commander please retry with connect button"}                              | Connexion with commander not etablished  |
| /free/       | GET     | | 200  | {'compliant': 'True'}            | Robot is compliant     |
| /free/       | GET     | | 200  | {'compliant': 'False'}           | Robot is not compliant |
| /free/       | POST    |{'compliant': 'True'}     | 200  | {'compliant': 'True'}       | Robot has been set compliant |
| /free/       | POST    |{'compliant': 'False'}     | 200  | {'compliant': 'False'}       | Robot has been set not compliant |
| /free/       | POST    |None or incorrect    | 400  | Bad Request      | |
| /free/       | POST    | {'compliant' : 'Bool'}    | 404  | Not Found       | Robot is not alive |
| /point/add/simulation | POST | {} | 408 | {"Error": "Rviz doesn't send response"} | Rviz is not alive or very slow |
| /point/add/simulation | POST | {} | 200 | {"Success" : "Add cartesian point with id: int"} | A point has been recorded |
| /point/add/actual | POST | {} | 408 | {"Error": "MoveIt doesn't send response"} | MoveIt is not alive or very slow |
| /point/add/actual | POST | {} | 200 | {"Success" : "Add cartesian point with id: int"} | A point has been recorded |
| /point/get | GET | | 404 | {"Error": "No point have been recorded"} | ros parameters server (on the name "cartesianPoints") is empty  |
| /point/get | GET | | 200 | A json with all cartesian points | |
| /point/get/id | GET | | 404 | {"Error": "No point have been recorded"} |ros parameters server (on the name "cartesianPoints") doesn't contain point with this id |
| /point/get/id | GET | | 200 | A json with one point | |
| /point/delete | POST | {} | 200 | {"Success": "All points have been deleted"} | All points have been cleared or database was empty |
| /point/delete/id | POST | {} | 404 | {"Error": "No point match with id: int" | ros parameters server (on the name "cartesianPoints") doesn't contain point with this id  |
| /point/delete/id | POST | {} | 200 | {"Success": "Point have been deleted"} | Point has been deleted |
| /control/ | POST | an export of nodered tree | 200 | {"Success": "All behavior trees has been executed"} | |
| /control/ | POST | None | 400 | {"Error": "json is empty"} | Can't evaluate an empty file |
| /control/ | POST | an export of nodered tree| 400 | {"Error": "json contains 0 valid roots"} | Json is not empty but doesn't contain root block |
| /control/ | POST | an export of nodered tree| 400 | {"Error": "Tree with root id <id> is incorrect"} | Json contains a root block but associate tree is incorrect|
| /control/ | POST | an export of nodered tree| 400 | {"Error": "Block (or child of this block) with id <id> is incorrect"} | Json contains tree but one block is incorrect|
| /control/stop | GET | | 200 | {"Success": "Behavior tree has been stopped "} | Current behavior tree is stopped|
| /teleop/force-stop | POST | {} | 200 | "Success" | Robot stop |
| /teleop/forward | POST | {} | 200 | "Success" | Robot go forward |
| /teleop/backward | POST | {} | 200 | "Success" | Robot go backward |
| /teleop/left | POST | {} | 200 | "Success" | Robot go left |
| /teleop/right | POST | {} | 200 | "Success" | Robot go right |

## Tests

To launch test you need to launch rolling robot or a manipulator arm with one of these exemple commands :
```bash
# for manipulator arm
roslaunch ur3_moveit_config demo.launch config:=true

# for rolling robot
roslaunch turtlebot3_gazebo turtlebot3_house.launch
# in another terminal 
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```

Then launch ronoco

```bash
# for manipulator arm
roslaunch ronoco manipulator.launch commander:=manipulator compliant_mode:=manual

# for rolling robot
roslaunch ronoco rolling.launch mode:=rolling topic_move_base:=/move_base amcl_pose:=/amcl_pose cmd_vel:=/cmd_vel
```

Finally, launch test
```bash
cd $HOME/catkin_ws/src/ronoco/ronoco-vm
python3 tests/launcher.py
```