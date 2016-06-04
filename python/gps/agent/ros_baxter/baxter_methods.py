# Importing Baxter stuff:
#!/usr/bin/env python
import argparse
import random

import rospy
import roslib; roslib.load_manifest('gps_agent_pkg')

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics


# Proposed joint name order of joint commands coming from the policy... not sure if this matters.
baxter_joint_name_list = ['right_e0','right_s0','right_s1','right_w0','right_e1','right_w1','right_w2']

class BaxterMethods:

    def __init__(self):
        self._setup_baxter_world()
        self.limb = baxter_interface.Limb('right')
        self.kin = baxter_kinematics('right')

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
        self.limb.set_joint_positions(baxter_list_to_dict(joint_angles_list),True)

    def set_baxter_joint_velocities(self, joint_angles_list):
        if len(joint_angles_list) != 7:
            print "The number of joint angles passed to baxter are: " + str(len(joint_angles_list))
        self.limb.set_joint_velocities(baxter_list_to_dict(joint_angles_list))

    def set_baxter_joint_positions(self, joint_angles_list):
        joint_dict = baxter_list_to_dict(joint_angles_list)
        self.limb.move_to_joint_positions(joint_dict)

    def get_baxter_joint_angles_positions(self):
        observed_joint_angles_dict = self.limb.joint_angles()
        if len(observed_joint_angles_dict) != 7:
            print "The number of joint angles taken from baxter are: " + str(len(observed_joint_angles_dict))
        return baxter_dict_to_list(observed_joint_angles_dict)

    def get_baxter_joint_angles_velocities(self):
        observed_joint_velocities_dict = self.limb.joint_velocities()
        if len(observed_joint_velocities_dict) != 7:
            print "The number of joint angles taken from baxter are: " + str(len(observed_joint_velocities_dict))
        return baxter_dict_to_list(observed_joint_velocities_dict)

    def get_baxter_end_effector_pose(self):
        pose = self.limb.endpoint_pose()
        return list(pose['position'])  + [0]*3 #+ list(pose['orientation'])[:3]

    def get_baxter_end_effector_velocity(self):
        pose = self.limb.endpoint_velocity()
        return list(pose['linear']) + list(pose['angular'])

    def get_baxter_end_effector_jacobian(self):
        return self.kin.jacobian()



def baxter_dict_to_list(dictionary):
    joint_list = []
    for i in range(len(baxter_joint_name_list)):
        joint_list.append(dictionary[baxter_joint_name_list[i]])
    return joint_list

def baxter_list_to_dict(joint_list):
    joint_dict = {}
    for i in range(len(joint_list)):
        joint_dict[baxter_joint_name_list[i]] = joint_list[i]
    return joint_dict
