# Ronoco

Avec ROS, créer un système robotisé revient à développer des programmes en C++ ou Python, rendant ROS inaccessible à
tous les experts non-développeurs : ingénieurs process, ingénieurs industrialisation ...

Écrire du code devient en effet indispensable pour les robots complexes.

Cependant depuis plusieurs années une méthode alternative performe : le no-code. Cela consiste à concevoir des 
programmes
informatiques sans écrire de code, souvent via une interface graphique. Ce projet consiste à développer un prototype
de "ROS no-code" permettant de programmer des robots manipulateurs sous MoveIt et des robots roulants.

Ronoco se connecte à l'écosystème ROS et générera des messages standards ROS, en conservant l'ADN interopérable de ROS (
robot-agnostic).

Le projet ronoco se décompose en trois modules distincts remplissant tous les trois des rôles différents et 
complémentaires permettant d'obtenir un interface graphique pour concevoir des programmes de manipulation ROS. 

Les trois modules sont **ronoco-vm** un package ROS constitué d'une API flask permettant de relier l'interface 
graphique ainsi que MoveIt et Rviz. Le deuxième est **ronoco-ui** un client web permettant  de contrôler le robot 
graphiquement (enregistrer des positions, lancer le programme, arrêt etc). Le dernier est TODO

## Ronoco-vm

[Documentation du module](ronoco-vm/README.md)


