# Getting started
The installation is divide into two parts. First it is necessary to install ROS Noetic and then to install the different modules of ronoco. An installation guide for ROS noetic for Ubuntu is provided below. Refer to [the official page](http://wiki.ros.org/noetic/Installation) for more details

## Ubuntu install of ROS Noetic

### Installation

Configure your Ubuntu to allow "restricted", "universe" and "multiverse". You can
[follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

Set up your computer to accept software from packages.ros.org

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
> /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys

```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \ 
sudo apt-key add -
```

First, make sur your Debian package index is up-to-date then install ROS

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```

### Environment setup

You must source this script in every **bash** terminal you use ROS in

```bash
source /opt/ros/noetic/setup.bash
````

If can be convenient to automatically source this script every time a new shell is launched. These commands will do that
for you
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

#or if you use zsh
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```



## Ronoco

### Step by step

Pour installer Ronoco il est nécessaire d'avoir les paquets suivants :
- npm
- python3
- ROS noetic (work also on ROS melodic)

Pour commencer il est nécessaire de cloner le projet dans le workspace catkin
```bash
cd $HOME/catkin_ws/src/
git clone https://github.com/Sdelpeuch/Ronoco.git
```

Il faut ensuite installer les trois modules constituant ronoco. Premièrement ronoco-nodered permettant à l'utilisateur
de définir les arbres de comportement du robot (voir [How to use it]() et [How to create a behaviour tree]()). Ce module
dépend de [Nodered]() il est donc nécessaire d'installer le framework puis la palette spécifique à Ronoco.

```bash
sudo npm install -g --unsafe-perm node-red
mkdir $HOME/.node-red/
cd $HOME/.node-red/
npm install $HOME/catkin_ws/src/Ronoco/ronoco-nodered/
```

Une fois Ronoco-nodered installé il est nécessaire d'installer le client web contenu dans Ronoco-ui

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-ui/
npm install
```

Finalement il ne reste plus qu'à installer le moteur de l'application contenu dans Ronoco-vm.

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-vm/
pip3 install -r requirements.txt
```

Il est nécessaire de télécharger un client socketio dans le dossier static de l'API

```bash
cd $HOME/catkin_ws/src/Ronoco/ronoco-vm/ronoco_vm/static/
mkdir socket.io
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js
wget https://github.com/socketio/socket.io/blob/master/client-dist/socket.io.js.map
```

Avant d'utiliser Ronoco il est nécessaire de compiler le workspace ROS

```bash
cd $HOME/catkin_ws/
catkin_make
source devel/setup.<bash/zsh>
```

Pour démarrer ronoco référez vous à la page [How to use it]()