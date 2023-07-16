#!/usr/bin/env python3

# //////////  Author details  //////////
# Name      : Gnanesh.k
# Contact   : gnaneshroyal254@gmail.com
# Linkedin  : https://www.linkedin.com/in/gnanesh-royal-374126213
# Github    : https://github.com/GNANESHROYAL/ROS-ROBOTICS.git

import numpy as np
import rospy
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# Function to calculate joint positions
def joint_position():
    # Initialize the ROS node
    rospy.init_node('ik', anonymous=False)
    
    # Create a publisher for joint states
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # Set the publishing rate
    rate = rospy.Rate(10)
    
    # Get the URDF robot description parameter
    my_param = rospy.get_param('robot_description')
    
    # Parse the URDF description
    robot = URDF.from_xml_string(my_param)
    
    # Get the joint names and origins
    j_n = [joint.name for joint in robot.joints]
    j_o = [joint.origin for joint in robot.joints]
    
    # Get the number of links
    n_l = len(robot.link_map)
    
    # Calculate the number of joints
    joints = len(robot.joints) - 3
    
    # Print the number of joints
    print(joints)
    
    # Calculate the distances between joint origins
    dist = []
    for i in range(1, len(j_n)):
        o1 = j_o[i - 1]
        o2 = j_o[i]
        di = np.sqrt(pow((o1.xyz[0] - o2.xyz[0]), 2) + pow((o1.xyz[1] - o2.xyz[1]), 2) + pow((o1.xyz[2] - o2.xyz[2]), 2))
        dist.append(di)
    
    # Convert distances to numpy array
    dist = np.array(dist)
    dist = dist.astype('float32')
    
    # Print the length of distances
    print(len(dist))
    
    # Calculate the sum of distances
    co = sum(dist)
    
    # Create an empty list to store joint angle ranges
    j_a_r = []
    
    # Remove the first joint name
    del j_n[0]
    
    # Iterate over the robot joints
    for joint in robot.joints:
        # Check if the joint type is revolute
        if joint.type == "revolute":
            # Generate a linearly spaced array for the joint limits
            j_l = np.linspace(joint.limit.lower, joint.limit.upper, 100)
            j_a_r.append(j_l)
    
    # Print the limits of the workspace
    print("limits x:({},{}) y:({},{}) z:({},{})".format(-co, co, -co, co, -co, 0))
    
    # Set the initial origin coordinates
    print("current origin: ", xc, yc, zc)
    
    # Start the main loop
    while True:
        # Get input coordinates from the user
        x, y, z = input("Enter coordinates: ").split()
        
        # Iterate twice (for x and y coordinates)
        for i in range(0, 2):
            
            # Function to calculate the Jacobian matrix
            def jacob(joints, thetas):
                # Create a zero matrix for the Jacobian
                j = np.zeros((3, len(dist)))
                
                # Create a zero matrix for the collision matrix
                co_mat = np.zeros((n_l, n_l))
                
                # Create arrays for trigonometric calculations
                trig = np.zeros(len(dist))
                trig1 = np.zeros(len(dist))
                trig2 = np.zeros(len(dist))
                
                # Calculate trigonometric values
                for i in range(joints):
                    trig[i] = -np.sin(np.sum(thetas[:i + 1]))
                    trig1[i] = np.cos(np.sum(thetas[:i + 1]))
                    trig2[i] = np.sin(np.sum(thetas[:i + 1]))
                
                # Calculate the Jacobian matrix
                j[0, :] = dist * trig
                j[1, :] = dist * trig1
                j[2, :] = dist * trig2
                
                # Calculate the pseudo-inverse of the Jacobian
                j = np.linalg.pinv(j)
                
                # Calculate the error between desired and current coordinates
                error = [xd - xc, yd - yc, zd - zc]
                error = np.array(error)
                error = error.reshape(3, 1)
                
                # Calculate the joint angles using the Jacobian
                thetas = np.dot(j, error)
                
                # Calculate the collision matrix
                for i in range(n_l):
                    for j in range(i + 1, n_l):
                        if np.linalg.norm([xd, yd, zd]) < 0.1:
                            co_mat[i, j] = 1
                            print(co_mat)
                
                return thetas
            
            # Convert input coordinates to float
            xd = float(x)
            yd = float(y)
            zd = float(z)
            
            # Check if input coordinates are within workspace limits
            if -co <= xd <= co and -co <= yd <= co and -co <= zd <= 0:
                # Calculate joint angles using the Jacobian
                k = jacob(joints, thetas)
                
                # Convert the joint angles to degrees
                thetas = (k * 180) / 3.14
                
                # Update the current origin coordinates
                xc = xd
                yc = yd
                zc = zd
                
                # Print limits and current origin after the first iteration
                if i == 1:
                    print("limits x:({},{}) y:({},{}) z:({},{})".format(-co, co, -co, co, -co, 0))
                    print("current origin: ", xc, yc, zc)
                
                # Publish joint states to the topic
                elif i == 0:
                    # Convert the joint angles to the Float64 type
                    k = k.astype('Float64')
                    
                    # Create a JointState message
                    j_s = JointState()
                    
                    # Set the joint positions
                    j_s.position = k
                    
                    # Set the joint names
                    j_s.name = j_n
                    
                    # Set the timestamp and frame ID for the message
                    j_s.header.stamp = rospy.Time.now()
                    j_s.header.frame_id = "mamulga_undadhu"
                    
                    # Print the joint angles
                    print(thetas)
                    
                    # Log the joint states
                    rospy.loginfo(j_s)
                    
                    # Publish the joint states
                    pub.publish(j_s)
                    
                    # Sleep for a while
                    rate.sleep()
            
            else:
                print("Error input!!!")
                continue

if __name__ == '__main__':
   try:
       joint_position()
   except rospy.ROSInterruptException:
        pass
