#!/usr/bin/env python3
import os
import shlex
import subprocess
from pathlib import Path
from time import sleep

home_directory = str(Path.home())
node_red = subprocess.Popen("node-red")
ronoco_ui = subprocess.Popen(shlex.split("npm start"), cwd= home_directory + "/catkin_ws/src/ronoco/ronoco-ui/")
ronoco_vm = os.system("cd $HOME/catkin_ws/src/ronoco/ronoco-vm/ && python3 ronoco_vm/run.py")
node_red.kill()
subprocess.Popen(shlex.split("npm stop"), cwd= home_directory + "/catkin_ws/src/ronoco/ronoco-ui/")
