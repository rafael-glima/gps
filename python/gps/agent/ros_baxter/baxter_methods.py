# Importing Baxter stuff:
#!/usr/bin/env python
import argparse
import random

import rospy
import roslib; roslib.load_manifest('gps_agent_pkg')

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

# Proposed joint name order of joint commands coming from the policy... not sure if this matters.
baxter_joint_name_list = ['right_e0','right_s0','right_s1','right_w0','right_e1','right_w1','right_w2']

class BaxterMethods:

    def __init__(self):
        self._setup_baxter_world()
        self.limb = baxter_interface.Limb('right')

    def _setup_baxter_world(self):
        print("Initializing node... ")
        rospy.init_node("rsdk_joint_position_keyboard")
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled

        def clean_shutdown():
            print("\nExiting example...")
            if not init_state:
                print("Disabling robot...")
                rs.disable()
        rospy.on_shutdown(clean_shutdown)

        print("Enabling robot... ")
        rs.enable()

        # map_keyboard()
        print("Done.")

    def set_baxter_joint_angles(self, joint_angles_list):
        if len(joint_angles_list) != 7:
            print "The number of joint angles passed to baxter are: " + str(len(joint_angles_list))
        joint_angles_dict = {}
        for i in range(len(joint_angles_list)):
            joint_angles_dict[baxter_joint_name_list[i]] = joint_angles_list[i]
        self.limb.set_joint_positions(joint_angles_dict,True)

    def get_baxter_joint_angles(self):
        observed_joint_angles = self.limb.joint_angles()
        if len(observed_joint_angles) != 7:
            print "The number of joint angles taken from baxter are: " + str(len(observed_joint_angles))
        return observed_joint_angles