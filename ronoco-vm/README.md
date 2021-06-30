# Ronoco-vm

Le module **ronovo-vm** est l'un des modules du projet [ronoco](../README.md). C'est un paquet ROS et une A     PI flask
proposant des services pour contrôler un robot sous ROS.

## Installation

Depuis le dossier `ronoco-vm̀` réaliser un `pip install requirements.txt`. Une fois les dépendances installées
l'application se lance via la commande `python3 flaskr/run.py` et tourne sur le `localhost:5000`.

:warning: Pour fonctionner un roscore et rviz doivent être en cours d'execution

## Documentation

| URL          | Method | Body | Return Code     |    Return          |      Description            |
|:-------------|:--------| :--- | :---- |:-----|:---------------------------------|
| /            | GET     | | 200  | {"Success": "Server is running"} |                        |
| /robot_state | GET     | | 200  | {'robot_state': True}            |                        |
| /robot_state | GET     | | 404  |                                  | Robot is not alive     |
| /shutdown    | GET     | |      |                                  | Shutdown flask server  |
| /free/       | GET     | | 200  | {'compliant': 'True'}            | Robot is compliant     |
| /free/       | GET     | | 200  | {'compliant': 'False'}           | Robot is not compliant |
| /free/       | POST    |{'compliant': 'True'}     | 200  | {'compliant': 'True'}       | Robot has been set compliant |
| /free/       | POST    |{'compliant': 'False'}     | 200  | {'compliant': 'False'}       | Robot has been set not compliant |
| /free/       | POST    |None or incorrect    | 400  | Bad Request      ||
| /free/       | POST    | {'compliant'} : 'Bool'    | 404  | Not Found       | Robot is not alive |
| /point/add/rviz | POST | {} | 408 | {"Error": "Rviz doesn't send response"} | Rviz is not alive or very slow |
| /point/add/rviz | POST | {} | 200 | {"Success" : "Add cartesian point with id: int"} | A point has been recorded |
| /point/add/free | POST | {} | 408 | {"Error": "MoveIt doesn't send response"} | MoveIt is not alive or very slow |
| /point/add/free | POST | {} | 200 | {"Success" : "Add cartesian point with id: int"} | A point has been recorded |
| /point/get | GET | | 404 | {"Error": "No point have been recorded"} | ros parameters server (on the name "cartesianPoints") is empty  |
| /point/get | GET | | 200 | A json with all cartesian points | |
| /point/get/<id> | GET | | 404 | {"Error": "No point have been recorded"} |ros parameters server (on the name "cartesianPoints") doesn't contain point with this id |
| /point/get/<id> | GET | | 200 | A json with one point | |
| /point/delete | POST | {} | 200 | {"Success": "All points have been deleted"} | All points have been cleared or database was empty |
| /point/delete/<id> | POST | {} | 404 | {"Error": "No point match with id: int" | ros parameters server (on the name "cartesianPoints") doesn't contain point with this id  |
| /point/delete/<id> | POST | {} | 200 | {"Success": "Point have been deleted"} | Point has been deleted |
| /move/ | POST | {"id": list, "mode": "plan"/"execute"/"infinite"} | 200 | {"Success": "Action has been realized"} | |
| /move/ | POST | {"id": list, "mode": "plan"/"execute"/"infinite"} | 408 | {"Error": "Rviz doesn't send response"} | Rviz is not alive or very slow |
| /move/ | POST | {"id": list, "mode": None or Unknow} | 400 | {"Error": "Incorrect mode"} | Allowed mode are plan, execute or infinite |
| /move/ | POST | {"id": None or Unknow, "mode" : "plan"/"execute"/"infinite"} | 404 | {"Error": "Incorrect id"} | One of id in the list hasn't been recorded|


