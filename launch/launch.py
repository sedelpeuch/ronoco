#!/usr/bin/env python3
import os
import shlex
import subprocess
from pathlib import Path

import rospy

ronoco_path = rospy.get_param("ronoco_path")
editor = rospy.get_param("editor")
if editor == "nodered":
    node_red = subprocess.Popen("node-red")
elif editor == "blockly":
    ronoco_blockly = subprocess.Popen(shlex.split("npm start"), cwd=ronoco_path + "/ronoco-blockly/")
ronoco_ui = subprocess.Popen(shlex.split("npm start"), cwd=ronoco_path + "/ronoco-ui/")
ronoco_vm = os.system("cd " + ronoco_path + "/ronoco-vm/ && python3 ronoco_vm/run.py")
node_red.kill()
subprocess.Popen(shlex.split("npm stop"), cwd = ronoco_path + "/ronoco-ui/")
subprocess.Popen(shlex.split("npm stop"), cwd = ronoco_path + "/ronoco-blockly/")
