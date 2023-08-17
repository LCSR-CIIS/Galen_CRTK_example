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

    def run_move_cp(self):
        # Not yet implemented for Galen robot
        # if not self.enable(60):
        #     print("Unable to enable the device, make sure it is connected.")
        #     return
        # to force creating an initial setpoint but Galen should really be computing this based on last setpoint_jp
        print('initializing setpoint_cp from measured_cp')
        self.move_cp(self.measured_cp()).wait()
        # time.sleep(1.0)

        # create a new goal starting with current position
        start_cp = PyKDL.Frame()
        start_cp.p = self.measured_cp().p
        start_cp.M = self.measured_cp().M
        goal = PyKDL.Frame()
        amplitude = 20.0 # 2 centimeters

        # first move
        goal.p[0] = start_cp.p[0] + amplitude
        goal.p[1] = start_cp.p[1] + amplitude
        goal.p[2] = start_cp.p[2]

        self.move_cp(goal).wait()
        print('first move initiated')
        print('first move completed')

        
        # second move
        goal.p[0] = start_cp.p[0] - amplitude
        goal.p[1] = start_cp.p[1] - amplitude
        print('initiating second move')
        self.move_cp(goal).wait()
        print('second move completed')
        # back to starting point
        goal.p[0] = start_cp.p[0]
        goal.p[1] = start_cp.p[1]
        print('initiating third move')
        self.move_cp(goal).wait()
        print('third move completed')

# use the class now, i.e. main program
if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
    else:
        example = crtk_move_cp_example()
        example.configure(sys.argv[1])
        example.run_move_cp()

