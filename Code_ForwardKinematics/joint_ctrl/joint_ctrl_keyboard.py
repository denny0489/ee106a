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
import math
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

set_offset = True
offset = 0.0

def set_j(limb, joint_names, data):
	
    #ORDER IS YAW PITCH ROLL
    # data = IMU1, IMU2, IMU3
    # IMU1 = black = wrist_yaw, wrist_pitch, wrist_roll
    # IMU2 = green = __ , elbow_pitch, elbow_roll
    # IMU3 = white = __, shoulder_pitch, shoulder_roll

    N = 7
    joint_command = {}

    print(data)

    yaw = data[0] - 150
    if yaw > 180:
        yaw -= 360
    pitch = data[1]
    roll = data[2]

    yaw2 = data[3]
    pitch2 = data[4]
    roll2 = data[5]

    yaw3 = data[6]
    pitch3 = data[7]
    roll3 = data[8]
    

    yawRad = -math.radians(yaw)
    pitchRad = math.radians(pitch)
    rollRad = -math.radians(roll)

    pitchRad2 = -math.radians(pitch2)
    rollRad2 = math.radians(roll2)

    pitchRad3 = -math.radians(pitch3)
    rollRad3 = math.radians(roll3)

    if set_offset:
        offset = -yawRad
        set_offset = False

    yawRad = yawRad + offset

    # joint names = left_s0, left_s1, left_e0, left_e1, left_w0, left_w1, left_w2

    joint_command[joint_names[6]] = rollRad
    joint_command[joint_names[5]] = pitchRad

    joint_command[joint_names[4]] = rollRad2
    joint_command[joint_names[3]] = pitchRad2

    joint_command[joint_names[2]] = rollRad3
    joint_command[joint_names[1]] = pitchRad3

    joint_command[joint_names[0]] = yawRad

    #joint_command[joint_names[6]] = 0.0
    #joint_command[joint_names[5]] = 0.0
    #joint_command[joint_names[4]] = 0.0
    #joint_command[joint_names[3]] = 0.0
    #joint_command[joint_names[2]] = 0.0
    #joint_command[joint_names[1]] = 0.0
    #joint_command[joint_names[0]] = 0.0


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

def map_keyboard():
    limb = baxter_interface.Limb('left')

    try:
        gripper = baxter_interface.gripper(side + '_gripper')
    except Exception as e:
        print (e)
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
        exit()


    if (len(arduino.readline()) == 0):
        print ("No data from arduino")
        exit()

    # Data from Arduino is a single string
    # Data needs to be decoded, split, have start and end strings removed, and converted to floats


	# Set joint states 
    
    done = False

    # calibration: set Baxter to 0,0,0,0,0,0,0

    # set joint states using data from Arduino
    while not done and not rospy.is_shutdown():
        data1=[]
        data1 = arduino.readline().decode("utf-8", "ignore").split(" ")

        # attempt to convert string from Arduino into floats
        try:
            #print(data1)
            data = [float(i) for i in data1]
        # if float conversion fails skip this iteration
            
        except:
            print("Bad data, cannot be converted to float")
            continue
        if(len(data) < 9):
            continue
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

    set_offset = True

    # valid_limbs = rs.joint_names()
    # if not valid_limbs:
    #     rp.log_message(("Cannot detect any limb parameters on this robot. "
    #                     "Exiting."), "ERROR")
    #     return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    # parser.add_argument(
    #     "-l", "--limb", dest="limb", default=valid_limbs[0],
    #     choices=valid_limbs,
    #     help="Limb on which to run the joint position keyboard example"
    # )
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
    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()
