#!/usr/bin/env python

# Author: Hisashi Ishida
# Date: 2023-03-31
#
# Copyright (c) 2018-2023 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
# Released under MIT License

# To communicate with the Galen using ROS topics, run following command:
# > rosrun galen_crtk_telop_example.py /REMS/Research

import crtk
import math
import sys
import rospy
import numpy
import PyKDL

if sys.version_info.major < 3:
    input = raw_input

# example of application using device.py
class galen_crtk_teleop_example:

    # configuration
    def configure(self, device_namespace):
        # ROS initialization
        if not rospy.get_node_uri():
            rospy.init_node('galen_crtk_teleop_example', anonymous = True, log_level = rospy.WARN)

        print(rospy.get_caller_id() + ' -> configuring crtk_device_test for: ' + device_namespace)
        # populate this class with all the ROS topics we need
        self.crtk_utils = crtk.utils(self, device_namespace)
        self.crtk_utils.add_operating_state()
        self.crtk_utils.add_measured_cp()
        self.crtk_utils.add_measured_cf()
        self.crtk_utils.add_move_cp()
        

        #############################
        # Parameters
        #############################
        self.rate = 500    # aiming for 200 Hz

        # Define Start position
        self.start_cp = PyKDL.Frame()
        self.start_cp.p = self.measured_cp().p # Vector()
        self.start_cp.M = self.measured_cp().M # self.start_cp.M.DoRotY(math.pi).DoRotX()

        # for one trajectory
        self.duration = 5 # 5 seconds
        self.drill_motion= Vector(40., 0., 0.) # Move 4 cm along x-axis
        self.z_motion= Vector(0., 0., -2.0) # Move - 0.2 cm along z-axis

        # Transition to next motion
        self.next_motion = Vector(-40., 20., 0.) # Move -4 cm along x-axis and 2 cm along y axis
        self.samples = self.duration * self.rate

        self.num_stroke = 4

    # main loop
    def run(self):
        # Not yet implemented for Galen robot
        # if not self.enable(60):
        #     print("Unable to enable the device, make sure it is connected.")
        #     return

        self.running = True
        while (self.running):
            print ('\n- q: quit\n- p: print position, velocity\n- t: position based teleop (10s)')
            answer = input('Enter your choice and [enter] to continue\n')
            if answer == 'q':
                self.running = False
            elif answer == 'p':
                self.run_print()
            elif answer == 't':
                self.run_move_cp()
            else:
                print('Invalid choice\n')

    # print positions
    def run_print(self):
        print("Galen robot position")
        print(self.measured_cp().p)

        # You can add more if you want
        # print (self.measured_cf())

    # position based teleop
    def run_move_cp(self):

        goal_cp = PyKDL.Frame()
        goal_cp.p = self.start_cp.p
        goal_cp.M = self.start_cp.M # The robot will maintatin the rotation
        
        for stroke in range(num_stroke):
            # Start Drilling
            goal_cp.p = goal_cp.p + self.z_motion
            self.move_cp(goal).wait()
            
            # loop
            for i in range(self.samples):
                goal_cp.p = goal_cp.p + i/self.samples * self.drill_motion
                self.move_cp(goal)
                rospy.sleep(1.0/self.rate)

            # Ready for next motion
            goal_cp.p = goal_cp.p - self.z_motion
            self.move_cp(goal).wait()

            goal_cp.p = goal_cp.p + self.next_motion
            self.move_cp(goal).wait()

            handle.wait() # wait 2 seconding for each motion
            

# use the class now, i.e. main program
if __name__ == '__main__':
    try:
        if (len(sys.argv) != 2):
            print(sys.argv[0], ' requires one argument, i.e. crtk device namespace')
        else:
            example = galen_crtk_teleop_example()
            example.configure(sys.argv[1])
            example.run_move_cp()

    except rospy.ROSInterruptException:
        pass
