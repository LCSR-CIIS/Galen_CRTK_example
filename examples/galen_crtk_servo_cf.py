#!/usr/bin/env python

# Author: Hisashi Ishida, Anton Deguet
# Created on: 2024-01-18
#
# Copyright (c) 2015-2024 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# To communicate with the robot using ROS topics:
# > rosrun crtk_python_client crtk_arm_test.py <robot-name> (ex. /REMS/Research/compliance)

import crtk
import math
import sys
import rospy
import numpy as np
import PyKDL


# example of application using device.py
class crtk_servo_cf_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('crtk_servo_cf_example', anonymous = True, log_level = rospy.WARN)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_servo_cf()
        # for all examples
        self.duration = 10 # 10 seconds
        self.rate = 200    # aiming for 200 Hz
        self.samples = self.duration * self.rate

    def run_servo_cf(self):
        # Not yet implemented for Galen robot
        # if not self.enable(60):
        #     print("Unable to enable the device, make sure it is connected.")
        #     return

        # create a new goal with constant speed
        for i in range(self.samples):
            vel = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0]) # 1 N along z direction
            self.servo_cf(vel)
            rospy.sleep(1.0 / self.rate)

# use the class now, i.e. main program
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        else:
            example = crtk_servo_cf_example()
            example.configure(sys.argv[1])
            example.run_servo_cf()

    except rospy.ROSInterruptException:
        pass
