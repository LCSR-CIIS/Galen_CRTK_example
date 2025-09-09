#!/usr/bin/env python

# Author: Hisashi Ishida
# Author: Yu-Heng Deng
# Created on: 2023-08-17
#
# Copyright (c) 2015-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# Start a single arm using
# > rosrun dvrk_robot dvrk_console_json -j <console-file>

# To communicate with the arm using ROS topics, see the python based example dvrk_arm_test.py:
# > rosrun crtk_python_client crtk_arm_test.py <arm-name>

import crtk
import math
import sys
import time
import rospy
import numpy
import PyKDL

from pathlib import Path
from typing import List

# example of application using device.py
class crtk_move_cp_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        self.ral = crtk.ral('crtk_move_cp_example', device_namespace)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.ral('crtk_move_cp_example', device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_setpoint_cp()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_move_cp()
        self.ral.check_connections()
        self.tracjectory =[]

    def load_trajectory(self, file_path: Path):
        """
        Load poses.txt produced by the extractor scripts.
        Returns:
        - frame (PyKDL.Frame)
        """

        path = Path(file_path)
        # Read all lines, split last column (frame_id) safely even if empty
        with path.open("r") as f:
            for line in f:
                if not line.strip():
                    continue
                parts = line.rstrip("\n").split(" ")
                # Expect at least 10 tokens: sec nsec x y z qx qy qz qw (frame_id optional but we write it)
                if len(parts) < 10:
                    raise ValueError(f"Malformed line (need >=10 columns): {line}")
                sec  = int(parts[0]); nsec = int(parts[1])
                x,y,z = map(float, parts[2:5])
                qx,qy,qz,qw = map(float, parts[5:9])
                frame_id = " ".join(parts[9:])  # allow spaces in frame_id just in case
                frame = PyKDL.Frame()
                frame.p = (x, y, z)
                self.tracjectory.append(frame)


    def run_move_cp(self):
        # Not yet implemented for Galen robot
        # if not self.enable(60):
        #     print("Unable to enable the device, make sure it is connected.")
        #     return
        # to force creating an initial setpoint but Galen should really be computing this based on last setpoint_jp
        
        for goal in self.tracjectory:
            self.move_cp(goal).wait()

# use the class now, i.e. main program
if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
    else:
        example = crtk_move_cp_example()
        example.configure(sys.argv[1])
        example.run_move_cp()

