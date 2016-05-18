# Importing Baxter stuff:
#!/usr/bin/env python
import argparse
import random

import rospy
import roslib; roslib.load_manifest('gps_agent_pkg')

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

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
        self.limb.set_joint_positions(joint_angles_dict,True)

    def get_baxter_joint_angles(self):
        return self.limb.joint_angles()