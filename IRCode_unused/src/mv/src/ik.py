#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from baxter_interface import gripper as robot_gripper
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
import math
import serial #To install

# https://arduino.stackexchange.com/questions/45874/arduino-serial-communication-with-python-sending-an-array

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():   
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
	
        # first element has an additional "u'" on the front that needs to be stripped
        data1[0] = data1[0][2:]
	
        # last element has an additional "\r\n'" on the end that needs to be stripped
        len_last = len(data1[6])
        data1[6] = data1[6][:len_last-5]
	
        # attempt to convert string from Arduino into floats
        try:
            data = [float(i) for i in data1]
        # if float conversion fails skip this iteration
        except:
            print("Bad data, cannot be converted to float")
            continue

        # print("x1, y1, x2, y2", data[0],data[1],data[2],data[3],data[4],data[5])
        # data from Arduino is of the form [x1, y1, x2, y2, imu_yaw(z), imu_pitch(y), imu_roll(x)]
        # x_meters = x_pixels *0.609 / 1000
        # y_meters = y_pixels *0.609 / 750
        # may not need 0.609 term

        # -x1_world 		= y_baxter		in pixels
        pos_y = data[0] / 1000
	
        # -(1/2)(y1 + y2)_world = z_baxter		in pixels
        pos_z = (1/2)*(data[1] + data[3]) / 750

        # x2_world 		= x_baxter		in pixels
        pos_x = data[2] / 1000

        # roll, pitch in degrees [-180, 180]
        # roll sign is inverted and needs to be flipped
        # yaw in degrees [0, 360]
        roll = math.radians(data[6])
        pitch = math.radians(data[5])
        yaw = data[4]
	
        # yaw is off by 140 degrees
        # yaw is given in terms of [0, 360] and we want [-180, 180]
        yaw -= 140
        if yaw > 180:
            yaw -= 360
        yaw = math.radians(yaw)
	
	
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        # x is forward/backward
        request.ik_request.pose_stamped.pose.position.x = pos_x
        # y is left/right
        request.ik_request.pose_stamped.pose.position.y = pos_y
        # z is up/down
        request.ik_request.pose_stamped.pose.position.z = pos_z

        # orientation given from IMU in radians (roll, pitch, yaw)
        # convert from (roll, pitch, yaw) into quaternion
        orient_quat = quaternion_from_euler(roll, pitch, yaw)
        # x is roll
        request.ik_request.pose_stamped.pose.orientation.x = orient_quat[0]
        # y is pitch
        request.ik_request.pose_stamped.pose.orientation.y = orient_quat[1]
        # z is yaw
        request.ik_request.pose_stamped.pose.orientation.z = orient_quat[2]
        # negate w for inverse
        request.ik_request.pose_stamped.pose.orientation.w = orient_quat[3]

        print("x:, y:, z:", pos_x, pos_y, pos_z, "orientation", orient_quat[0], orient_quat[1], orient_quat[2], orient_quat[3])        

        try:
            #Send the request to the service
            response = compute_ik(request)

            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            group.go()
            
            # Code for gripper, to be used with flex-sensor
            #left_gripper = robot_gripper.Gripper('left')
            # Calibrate the gripper (other commands won't work unless you do this first)
            #print('Calibrating...')
            #left_gripper.calibrate()
            #rospy.sleep(2.0)

            # If flex sensor detects close:
            # Close the left gripper
            #print('Closing...')
            #left_gripper.close()
            #rospy.sleep(1.0)

            # If flex sensor detects open
            # open the left gripper
            #print('Opening...')
            #left_gripper.open()
            #rospy.sleep(1.0)
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()
