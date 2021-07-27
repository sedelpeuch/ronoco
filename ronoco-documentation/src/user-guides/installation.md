# Getting started
The installation is divide into two parts. First it is necessary to install ROS Noetic and then to install the different modules of ronoco. An installation guide for ROS noetic for Ubuntu is provided below. Refer to [the official page](http://wiki.ros.org/noetic/Installation) for more details

## Ubuntu install of ROS Noetic

### Installation

Configure your Ubuntu to allow "restricted", "universe" and "multiverse". You can
[follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

Setup your computer to accept software from packages.ros.org

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
