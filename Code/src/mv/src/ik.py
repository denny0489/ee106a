#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
from baxter_interface import gripper as robot_gripper
import serial #To install

# https://arduino.stackexchange.com/questions/45874/arduino-serial-communication-with-python-sending-an-array

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        try: 
            arduino = serial.Serial("/dev/ttyACMO", timeout = 1)
        except:
            print("Error when getting data from arduino")
            continue;

        if (arduino.readLine().length() == 0):
            print ("No data from arduino")
            continue

        #If decoding is needed
        # decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
        # print(decoded_bytes)
        data = []
        data = str(arduino.readLine()).split(" ")

        # data format: 
        # space separated data 
        # length = 6 
        # [position.x position.y position.z orientation.x orientation.y orientation.z]

        # Get input from Arduino
        # final position x, y, z, orientation x, y, z
        # JSON format?

        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.711
        request.ik_request.pose_stamped.pose.position.y = 0.743
        request.ik_request.pose_stamped.pose.position.z = -0.059      
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        #Construct the request
        request2 = GetPositionIKRequest()
        request2.ik_request.group_name = "left_arm"
        request2.ik_request.ik_link_name = "left_gripper"
        request2.ik_request.attempts = 20
        request2.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request2.ik_request.pose_stamped.pose.position.x = 0.786
        request2.ik_request.pose_stamped.pose.position.y = 0.086
        request2.ik_request.pose_stamped.pose.position.z = -0.032      
        request2.ik_request.pose_stamped.pose.orientation.x = 0.0
        request2.ik_request.pose_stamped.pose.orientation.y = 1.0
        request2.ik_request.pose_stamped.pose.orientation.z = 0.0
        request2.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            
            #Print the response HERE
            print(response)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            group.go()

            left_gripper = robot_gripper.Gripper('left')
            #Calibrate the gripper (other commands won't work unless you do this first)
            print('Calibrating...')
            left_gripper.calibrate()
            rospy.sleep(2.0)

            #Close the right gripper
            print('Closing...')
            left_gripper.close()
            rospy.sleep(1.0)



            # Setting position and orientation target
            group.set_pose_target(request2.ik_request.pose_stamped)
            group.go()

            #Close the right gripper
            print('Opening...')
            left_gripper.open()
            rospy.sleep(1.0)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

