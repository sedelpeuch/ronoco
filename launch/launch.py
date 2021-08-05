#!/usr/bin/env python3
import os
import shlex
import subprocess
from pathlib import Path

import rospy

ronoco_path = rospy.get_param("ronoco_path")
node_red = subprocess.Popen("node-red")
ronoco_ui = subprocess.Popen(shlex.split("npm start"), cwd=ronoco_path + "/ronoco-ui/")
ronoco_vm = os.system("cd " + ronoco_path + "/ronoco-vm/ && python3 ronoco_vm/run.py")
node_red.kill()
subprocess.Popen(shlex.split("npm stop"), cwd = ronoco_path + "/ronoco-ui/")
