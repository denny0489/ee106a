#!/usr/bin/env python

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse
import time
import rospy
import serial

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def map_keyboard(side):
    limb = baxter_interface.Limb(side)

    try:
        gripper = baxter_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()
	
	# attempt to connect to Arduinio
    try: 
        arduino = serial.Serial("/dev/ttyACM0", 115200)
    except:
        print("Error when getting data from Arduino")
        continue;

    #while True:
    #    print("Data from arduino = " + arduino.readline().decode("utf-8","ignore"))

    if (len(arduino.readline()) == 0):
        print ("No data from arduino")
        continue

    # Data from Arduino is a single string
    # Data needs to be decoded, split, have start and end strings removed, and converted to floats
    data1=[]
    data1 = arduino.readline().decode("utf-8", "ignore").split(" ")

    # attempt to convert string from Arduino into floats
    try:
        data = [float(i) for i in data1]
    # if float conversion fails skip this iteration
    except:
        print("Bad data, cannot be converted to float")
        continue

	# Set joint states 
    def set_j(limb, joint_names, data):
        N = 7
        joint_command = {}
        for i in range(N):
            current_position = limb.joint_angle(joint_names[i])
            joint_command[joint_names[i]] = float(data[i]
        limb.set_joint_positions(joint_command)
        #r.sleep()

    # currently unused, left in in case we want to use gripper later
    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    # set joint states using data from Arduino
    while not done and not rospy.is_shutdown():
        r = rospy.Rate(10)
        set_j(limb, joints, data)
        r.sleep()

def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = baxter_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
