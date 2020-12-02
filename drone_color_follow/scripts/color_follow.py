#! /usr/bin/env python
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2

vel_msg = Twist()
ref_radius = 30
ref_center = (160,160)
radius_error = 0
center_error = (0,0)
def image_callback(data):
    global image
    global radius_error
    global center_error
    global ref_radius
    global ref_center
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data,desired_encoding = "bgr8")

    low_blue = np.array([94, 80, 2])
    high_blue = np.array([126, 255, 255])
    hsv_frame = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, low_blue, high_blue)
    _, contours, h = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    image = cv2.drawContours(image, contours, -1, (0,255,255), 3)

    if len(contours)>0:
        cnt = max(contours,key=cv2.contourArea)
        center,radius = cv2.minEnclosingCircle(cnt)
        radius = int(radius)
        center = (int(center[0]),int(center[1]))
        cv2.circle(image,center,radius,[255,0,0],3)
        # print((radius,center))

        radius_error = ref_radius - radius
        center_error = (ref_center[0]-center[0],ref_center[1]-center[1])



    cv2.imshow('mask',mask)
    cv2.imshow('image',image)

    # blue = cv2.bitwise_and(frame, frame, mask=blue_mask)

    cv2.waitKey(1)

rospy.init_node('car_follower',anonymous=True)
try:
    image_sub = rospy.Subscriber('/uav1/front_cam/camera/image',Image,image_callback)
    vel_pub = rospy.Publisher('/uav1/cmd_vel',Twist,queue_size=10)
    pid_x = [0.01,0,0] #0.01,0,0
    pid_z = [0.01,0,0] #0.01,0,0
    pid_az = [0.01,0,0] #0.01,0,0
    rate = rospy.Rate(10)
    prev_radius_error = 0
    prev_centerx_error = 0
    prev_centery_error = 0

    total_radius_error = 0
    total_centerx_error = 0
    total_centery_error = 0
    while not rospy.is_shutdown():
        print((radius_error,center_error))

        if abs(radius_error) < 5:
            total_radius_error = 0
        if abs(center_error[0]) < 5:
            total_centerx_error = 0
        if abs(center_error[1]) < 5:
            total_centery_error = 0

        vel_x = pid_x[0]*radius_error + pid_x[1]*total_radius_error + pid_x[2]*(prev_radius_error-radius_error)
        vel_z = pid_z[0]*center_error[1] + pid_z[1]*total_centery_error + pid_z[2]*(prev_centery_error-center_error[1])
        vel_az = pid_az[0]*center_error[0] + pid_az[1]*total_centerx_error + pid_az[2]*(prev_centerx_error-center_error[0])

        prev_radius_error += radius_error
        prev_centerx_error += center_error[0]
        prev_centery_error += center_error[1]

        # if vel_x > 0.5:
        #     vel_x = 0.5
        # elif vel_x < -0.5:
        #     vel_x = -0.5
        #
        # if vel_z > 0.5:
        #     vel_z = 0.5
        # elif vel_z < -0.5:
        #     vel_z = -0.5

        vel_msg.linear.x = vel_x
        vel_msg.linear.z = vel_z
        vel_msg.angular.z = vel_az

        vel_pub.publish(vel_msg)

        rate.sleep()
except Exception as e:
    print('Error in launching node')
    print(e)
