#! /usr/bin/env python
import rospy
import sys

from pyquaternion import Quaternion

# from drone_waypoints_tsp.msg import leader
from swarm_search.msg import leader
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

from geometry_msgs.msg import PoseStamped #Message type for pose information
from geometry_msgs.msg import Twist #Message type for velocity information

# to get information about the current leader
def checkLeader(data):
    global leaderInfo
    leaderInfo = data

# callback for the position data
def poseCallback(data):
    global uav1_pose
    uav1_pose = data

# callback for the leader's position
def leaderPoseCallback(data):
    global leader_pose
    leader_pose = data

# to read in image data and process it
def image_callback(data):
    global radius_error, center_error, visible

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data,desired_encoding = "bgr8")

    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    hsv_frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    contours, h = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    image = cv2.drawContours(image, contours, -1, (0,255,255), 3)

    if not contours:
        visible = False
    elif len(contours) > 0:
        cnt = max(contours,key=cv2.contourArea)
        center,radius = cv2.minEnclosingCircle(cnt)
        radius = int(radius)
        # switch to image PID loop if the radius of the circle is greater than a constant
        if radius > 5:
            visible = True
        center = (int(center[0]), int(center[1]))
        cv2.circle(image, center, radius, [255,0,0], 3)
        #print(radius)

        radius_error = ref_radius - radius
        center_error = (ref_center[0]-center[0], ref_center[1]-center[1])

    #cv2.imshow('mask', mask)
    cv2.imshow('image' + ns[-2], image)
    cv2.waitKey(1)

global uav1_pose #global value for drone position
global prev_time #global value for storing the time

ns = rospy.get_namespace()
N = int(ns[-2])

with open(sys.argv[1] + ns[:-1] + '_points.txt', 'r') as fp:
    out = fp.read()
    out = out.split(';')
    coords = [map(int, i.split()) for i in out[:-1]]
    path = map(int, out[-1].split())

print(coords, path)

uav1_pose = PoseStamped() #empty initialization
leader_pose = PoseStamped()

# Ros node initialization
rospy.init_node('bot_search', anonymous=True)
# Subscribe to the topic publishing position
rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, poseCallback)
image_sub = rospy.Subscriber('/front_cam/camera/image', Image, image_callback)
# Create a publisher for the velocities
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# Subscribe to the topic which contains details about the current leader
leader_sub = rospy.Subscriber('/leader_info', leader, checkLeader)

# PID values for the x,y and z axis. Change for better performance
pid_x = [0.5,0.001,0.01]
pid_y = [0.5,0.001,0.01]
pid_z = [0.5,0.001,0.01]

# variable to store the velocity to be published to the drone
vel_msg = Twist()

# variable to store information about the leader
# initialize it to false at start-up since the topic isn't being published
leaderInfo = leader()
leaderInfo.id = N
leaderInfo.found = False

# variables for tracking and detection
visible = False
ref_radius = 30
ref_center = (160,160)
radius_error = 0
center_error = (0,0)

# Rate at which to publish
rate = rospy.Rate(30)

# the errors to be used for PID
pose_error = [0, 0, 0]
prev_pose_error = [0, 0, 0]
prev_total_error = [0, 0, 0]

# destination position - (x,y,z)
i = 0
dest_pose = coords[path[i]]

# the current time as caluclated bt ROS -> Time from the start of the ros master
prev_time = rospy.get_rostime().nsecs # get in nano seconds

# loop that runs as long as the ros master runs and the bot isn't found yet
while not rospy.is_shutdown() and not (visible or leaderInfo.found):
    print(visible, dest_pose)

    if abs(pose_error[0]) < .05 and abs(pose_error[1]) < .05 and abs(pose_error[2]) < .05:
        i = (i + 1) % len(path)
        dest_pose = coords[path[i]]
        pose_error = [0, 0, 0]
        prev_pose_error = [0, 0, 0]
        prev_total_error = [0, 0, 0]

    #Calculate errors of the x,y and z axis
    pose_error = [dest_pose[0]-uav1_pose.pose.position.x,
                  dest_pose[1]-uav1_pose.pose.position.y,
                  dest_pose[2]-uav1_pose.pose.position.z]
    
    # set velocity values to 0
    vel_x = 0
    vel_y = 0
    vel_z = 0
    
    # get current time
    now = rospy.get_rostime().nsecs
    
    # calculate time difference from the previous loop
    diff = float(now - prev_time)/1e9
    
    # edge case: First loop - both now and prev_time are equal, So diff = 0. Make it the one loop time = 1/rate = 1/30
    if diff == 0:
        diff = 0.033

    #Preventing integral windup
    if pose_error[0] <= 0.005:
        prev_total_error[0] = 0

    if pose_error[1] <= 0.005:
        prev_total_error[1] = 0

    if pose_error[2] <= 0.005:
        prev_total_error[2] = 0

    #Calculating the pid values
    # o/p = Kp*error + Ki*sum of errors + kd*(prev_err*curr_err)/time
    vel_x = pid_x[0]*pose_error[0] + pid_x[2]*((prev_pose_error[0]-pose_error[0])/diff) + pid_x[1]*prev_total_error[0]
    vel_y = pid_y[0]*pose_error[1] + pid_y[2]*((prev_pose_error[1]-pose_error[1])/diff) + pid_y[1]*prev_total_error[1]
    vel_z = pid_z[0]*pose_error[2] + pid_z[2]*((prev_pose_error[2]-pose_error[2])/diff) + pid_z[1]*prev_total_error[2]

    #Update the errors for the integrals
    prev_total_error[2] += pose_error[2]
    prev_total_error[1] += pose_error[1]
    prev_total_error[0] += pose_error[0]

    prev_time = now
    
    # limit the x and y velocites to max 5m/s. Can be reduced if needed. Might have to change PID values for optimal performance
    if vel_x > 5:
        vel_x = 5
    if vel_y > 5:
        vel_y = 5

    #Setting the values to be published
    vel_msg.linear.x = vel_x
    vel_msg.linear.y = vel_y
    vel_msg.linear.z = vel_z
    
    #publish the data
    vel_pub.publish(vel_msg)
    
    #Set prev_error as the current_error to use in the next loop
    prev_pose_error = pose_error
    
    # Sleep for the rate time
    rate.sleep()

# reset velocity
vel_msg.linear.x = 0
vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_pub.publish(vel_msg)

im_pid_x = [0.05,5e-4,0] #0.01,0,0
im_pid_z = [0.05, 5e-4, 0] #0.01,0,0
im_pid_az = [0.01,0,0] #0.01,0,0

# set variables for PID based tracking of the object
prev_radius_error = 0
prev_centerx_error = 0
prev_centery_error = 0

total_radius_error = 0
total_centerx_error = 0
total_centery_error = 0

if visible and not leaderInfo.found:
    leaderInfo.id = N
    leaderInfo.found = True
    # Create a publisher for the leader's info
    leader_pub = rospy.Publisher('/leader_info', leader, queue_size=10)

    # loop that runs once the bot is found
    while not rospy.is_shutdown():
        print(radius_error, center_error)
        
        if abs(radius_error) < 5:
            total_radius_error = 0
        if abs(center_error[0]) < 5:
            total_centerx_error = 0
        if abs(center_error[1]) < 5:
            total_centery_error = 0

        vel_x = im_pid_x[0]*radius_error + im_pid_x[1]*total_radius_error + im_pid_x[2]*(prev_radius_error-radius_error)
        vel_z = im_pid_z[0]*center_error[1] + im_pid_z[1]*total_centery_error + im_pid_z[2]*(prev_centery_error-center_error[1])
        vel_az = im_pid_az[0]*center_error[0] + im_pid_az[1]*total_centerx_error + im_pid_az[2]*(prev_centerx_error-center_error[0])

        prev_radius_error += radius_error
        prev_centerx_error += center_error[0]
        prev_centery_error += center_error[1]

        vel_msg.linear.x = vel_x
        vel_msg.linear.z = vel_z
        vel_msg.angular.z = vel_az

        vel_pub.publish(vel_msg)
        leader_pub.publish(leaderInfo)

        rate.sleep()

elif leaderInfo.found and leaderInfo.id != N:
    # Subscribe to the leader's position if you're not the leader
    rospy.Subscriber('/drone' + str(leaderInfo.id) + '/ground_truth_to_tf/pose',
                     PoseStamped, leaderPoseCallback)
    # the errors to be used for PID
    pose_error = [0, 0, 0]
    prev_pose_error = [0, 0, 0]
    prev_total_error = [0, 0, 0]
    pid_x = [1.05, 0.017, 0.31]
    pid_y = [1.05, 0.017, 0.31]
    pid_z = [1.05, 0.017, 0.31]

    # the current time as caluclated bt ROS -> Time from the start of the ros master
    prev_time = rospy.get_rostime().nsecs # get in nano seconds

    # loop that runs as long as the ros master runs and the bot isn't found yet
    while not rospy.is_shutdown():
        q = Quaternion(leader_pose.pose.orientation.w, leader_pose.pose.orientation.x,
                       leader_pose.pose.orientation.y, leader_pose.pose.orientation.z)
        
        if N == 1:
            relative_pose = [2, 0, 0]
            dest_pose = q.rotate(relative_pose)
            dest_pose = [leader_pose.pose.position.x + dest_pose[0],
                         leader_pose.pose.position.y + dest_pose[1],
                         leader_pose.pose.position.z + dest_pose[2]]
        elif N == 2    :
            relative_pose = [0, 2, 0]
            dest_pose = q.rotate(relative_pose)
            dest_pose = [leader_pose.pose.position.x + dest_pose[0],
                         leader_pose.pose.position.y + dest_pose[1],
                         leader_pose.pose.position.z + dest_pose[2]]
        elif N == 3:
            relative_pose = [-2, 0, 0]
            dest_pose = q.rotate(relative_pose)
            dest_pose = [leader_pose.pose.position.x + dest_pose[0],
                         leader_pose.pose.position.y + dest_pose[1],
                         leader_pose.pose.position.z + dest_pose[2]]
        elif N == 4:
            relative_pose = [0, -2, 0]
            dest_pose = q.rotate(relative_pose)
            dest_pose = [leader_pose.pose.position.x + dest_pose[0],
                         leader_pose.pose.position.y + dest_pose[1],
                         leader_pose.pose.position.z + dest_pose[2]]

        #Calculate errors of the x,y and z axis
        pose_error = [dest_pose[0] - uav1_pose.pose.position.x,
                      dest_pose[1] - uav1_pose.pose.position.y,
                      dest_pose[2] - uav1_pose.pose.position.z]
        
        # set velocity values to 0
        vel_x = 0
        vel_y = 0
        vel_z = 0
        
        # get current time
        now = rospy.get_rostime().nsecs
        
        # calculate time difference from the previous loop
        diff = float(now - prev_time)/1e9
        
        # edge case: First loop - both now and prev_time are equal, So diff = 0. Make it the one loop time = 1/rate = 1/30
        if diff == 0:
            diff = 0.033

        #Preventing integral windup
        if pose_error[0] <= 0.005:
            prev_total_error[0] = 0

        if pose_error[1] <= 0.005:
            prev_total_error[1] = 0

        if pose_error[2] <= 0.005:
            prev_total_error[2] = 0

        #Calculating the pid values
        # o/p = Kp*error + Ki*sum of errors + kd*(prev_err*curr_err)/time
        vel_x = pid_x[0]*pose_error[0] + pid_x[2]*((prev_pose_error[0]-pose_error[0])/diff) + pid_x[1]*prev_total_error[0]
        vel_y = pid_y[0]*pose_error[1] + pid_y[2]*((prev_pose_error[1]-pose_error[1])/diff) + pid_y[1]*prev_total_error[1]
        vel_z = pid_z[0]*pose_error[2] + pid_z[2]*((prev_pose_error[2]-pose_error[2])/diff) + pid_z[1]*prev_total_error[2]

        #Update the errors for the integrals
        prev_total_error[2] += pose_error[2]
        prev_total_error[1] += pose_error[1]
        prev_total_error[0] += pose_error[0]

        prev_time = now
        
        # limit the x and y velocites to max 5m/s. Can be reduced if needed. Might have to change PID values for optimal performance
        if abs(vel_x) > 5:
            vel_x = 5
        if abs(vel_y) > 5:
            vel_y = 5

        qt = Quaternion(uav1_pose.pose.orientation.w, uav1_pose.pose.orientation.x,
                        uav1_pose.pose.orientation.y, uav1_pose.pose.orientation.z)
        qt_inv = qt.inverse
        vel_rotate = qt_inv.rotate([vel_x, vel_y, vel_z])

        #Setting the values to be published
        vel_msg.linear.x = vel_rotate[0]
        vel_msg.linear.y = vel_rotate[1]
        vel_msg.linear.z = vel_rotate[2]
        
        #publish the data
        vel_pub.publish(vel_msg)
        
        #Set prev_error as the current_error to use in the next loop
        prev_pose_error = pose_error
        
        # Sleep for the rate time
        rate.sleep()

print ("exit - hard")
