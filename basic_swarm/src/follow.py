#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from pyquaternion import Quaternion
import numpy as np

global uav1_pose
global uav2_pose
global uav3_pose
global uav4_pose
global uav5_pose
global quat
global uav1_vel
uav1_pose=PoseStamped()
uav2_pose=PoseStamped()
uav3_pose=PoseStamped()
uav4_pose=PoseStamped()
uav5_pose=PoseStamped()
quat = Quaternion()
uav1_vel = Twist()

pub1 = rospy.Publisher('/uav1/cmd_vel',Twist,queue_size=10)
pub2 = rospy.Publisher('/uav2/cmd_vel',Twist,queue_size=10)
pub3 = rospy.Publisher('/uav3/cmd_vel',Twist,queue_size=10)
pub4 = rospy.Publisher('/uav4/cmd_vel',Twist,queue_size=10)
pub5 = rospy.Publisher('/uav5/cmd_vel',Twist,queue_size=10)

pid_x = [1.05,0.017,0.31]
pid_y = [1.05,0.017,0.31]
pid_z = [1.05,0.017,0.31]

rate = rospy.Rate(30)


def uav1PoseCallback(data):
    global uav1_pose
    global quat
    uav1_pose = data
    quat = Quaternion(uav1_pose.pose.orientation.w,uav1_pose.pose.orientation.x,uav1_pose.pose.orientation.y,uav1_pose.pose.orientation.z)

    #print([data.pose.position.x,data.pose.position.y,data.pose.position.z])

def uav2PoseCallback(data):
    global uav2_pose
    uav2_pose = data

def uav3PoseCallback(data):
    global uav3_pose
    uav3_pose = data

def uav4PoseCallback(data):
    global uav4_pose
    uav4_pose = data

def uav5PoseCallback(data):
    global uav5_pose
    uav5_pose = data

def uav1VelCallback(data):
    global uav1_vel
    uav1_vel = data

def follow1():
    pose_error = []
    prev_pose_error = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
    prev_total_error = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

    uav2_pose_leader_frame = np.array([-2,0,0])
    uav3_pose_leader_frame = np.array([2,0,0])
    uav4_pose_leader_frame = np.array([0,2,0])
    uav5_pose_leader_frame = np.array([0,-2,0])

    prev_time = rospy.get_rostime().nsecs
    t = rospy.get_rostime().secs
    graph_data = open('points.txt','w')
    graph_data.close()
    graph_data = open('points.txt','a')
    i = 0

    while not rospy.is_shutdown():
        #print(type(prev_time))

        #rotate the vector from drone frame to global frame
        new_loc_uav2 = quat.rotate(uav2_pose_leader_frame)
        new_loc_uav3 = quat.rotate(uav3_pose_leader_frame)
        new_loc_uav4 = quat.rotate(uav4_pose_leader_frame)
        new_loc_uav5 = quat.rotate(uav5_pose_leader_frame)
        #set the location the ddrone has to reach in global frame
        uav2_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav2[0],uav1_pose.pose.position.y+new_loc_uav2[1],uav1_pose.pose.position.z+new_loc_uav2[2]]
        uav3_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav3[0],uav1_pose.pose.position.y+new_loc_uav3[1],uav1_pose.pose.position.z+new_loc_uav3[2]]
        uav4_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav4[0],uav1_pose.pose.position.y+new_loc_uav4[1],uav1_pose.pose.position.z+new_loc_uav4[2]]
        uav5_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav5[0],uav1_pose.pose.position.y+new_loc_uav5[1],uav1_pose.pose.position.z+new_loc_uav5[2]]


        #print(uav2_pose_global_frame)

        #Calculate errors
        pose_error = [[uav2_pose_global_frame[0]-uav2_pose.pose.position.x,uav2_pose_global_frame[1]-uav2_pose.pose.position.y,uav2_pose_global_frame[2]-uav2_pose.pose.position.z]
        ,[uav3_pose_global_frame[0]-uav3_pose.pose.position.x,uav3_pose_global_frame[1]-uav3_pose.pose.position.y,uav3_pose_global_frame[2]-uav3_pose.pose.position.z]
        ,[uav4_pose_global_frame[0]-uav4_pose.pose.position.x,uav4_pose_global_frame[1]-uav4_pose.pose.position.y,uav4_pose_global_frame[2]-uav4_pose.pose.position.z]
        ,[uav5_pose_global_frame[0]-uav5_pose.pose.position.x,uav5_pose_global_frame[1]-uav5_pose.pose.position.y,uav5_pose_global_frame[2]-uav5_pose.pose.position.z]]
        #print('error',pose_error)

        vel_x = 0
        vel_y = 0
        vel_z = 0


        now = rospy.get_rostime().nsecs
        diff = float(now - prev_time)/1e9
        #print(diff)
        if(diff == 0.0):
            diff = float(1/30)/1e9

        #Preventing integral windup
        if(pose_error[0][0]<=0.005):
            prev_total_error[0][0] = 0
        if(pose_error[1][0]<=0.005):
            prev_total_error[1][0] = 0
        if(pose_error[2][0]<=0.005):
            prev_total_error[2][0] = 0
        if(pose_error[3][0]<=0.005):
            prev_total_error[3][0] = 0


        if(pose_error[0][1]<=0.005):
            prev_total_error[0][1] = 0
        if(pose_error[1][1]<=0.005):
            prev_total_error[1][1] = 0
        if(pose_error[2][1]<=0.005):
            prev_total_error[2][1] = 0
        if(pose_error[3][1]<=0.005):
            prev_total_error[3][1] = 0

        if(pose_error[0][2]<=0.005):
            prev_total_error[0][2] = 0
        if(pose_error[1][2]<=0.005):
            prev_total_error[1][2] = 0
        if(pose_error[2][2]<=0.005):
            prev_total_error[2][2] = 0
        if(pose_error[3][2]<=0.005):
            prev_total_error[3][2] = 0

        #Calculating the pid values
        vel_x = [pid_x[0]*pose_error[0][0]+pid_x[1]*((prev_pose_error[0][0]-pose_error[0][0])/diff) + pid_x[1]*prev_total_error[0][0]
        ,pid_x[0]*pose_error[1][0]+pid_x[1]*((prev_pose_error[1][0]-pose_error[1][0])/diff) + pid_x[1]*prev_total_error[1][0]
        ,pid_x[0]*pose_error[2][0]+pid_x[1]*((prev_pose_error[2][0]-pose_error[2][0])/diff) + pid_x[1]*prev_total_error[2][0]
        ,pid_x[0]*pose_error[3][0]+pid_x[1]*((prev_pose_error[3][0]-pose_error[3][0])/diff) + pid_x[1]*prev_total_error[3][0]]

        vel_y = [pid_y[0]*pose_error[0][1]+pid_y[1]*((prev_pose_error[0][1]-pose_error[0][1])/diff) + pid_y[1]*prev_total_error[0][1]
        ,pid_y[0]*pose_error[1][1]+pid_y[1]*((prev_pose_error[1][1]-pose_error[1][1])/diff) + pid_y[1]*prev_total_error[1][1]
        ,pid_y[0]*pose_error[2][1]+pid_y[1]*((prev_pose_error[2][1]-pose_error[2][1])/diff) + pid_y[1]*prev_total_error[2][1]
        ,pid_y[0]*pose_error[3][1]+pid_y[1]*((prev_pose_error[3][1]-pose_error[3][1])/diff) + pid_y[1]*prev_total_error[3][1]]

        vel_z = [pid_z[0]*pose_error[0][2]+pid_z[2]*((prev_pose_error[0][2]-pose_error[0][2])/diff) + pid_z[1]*prev_total_error[0][2]
        ,pid_z[0]*pose_error[1][2]+pid_z[2]*((prev_pose_error[1][2]-pose_error[1][2])/diff) + pid_z[1]*prev_total_error[1][2]
        ,pid_z[0]*pose_error[2][2]+pid_z[2]*((prev_pose_error[2][2]-pose_error[2][2])/diff) + pid_z[1]*prev_total_error[2][2]
        ,pid_z[0]*pose_error[3][2]+pid_z[2]*((prev_pose_error[3][2]-pose_error[3][2])/diff) + pid_z[1]*prev_total_error[3][2]]

        print('Working')

        #Update the errors for the integrals
        prev_total_error[0][2] += pose_error[0][2]
        prev_total_error[1][2] += pose_error[1][2]
        prev_total_error[2][2] += pose_error[2][2]
        prev_total_error[3][2] += pose_error[3][2]

        prev_total_error[0][1] += pose_error[0][1]
        prev_total_error[1][1] += pose_error[1][1]
        prev_total_error[2][1] += pose_error[2][1]
        prev_total_error[3][1] += pose_error[3][1]

        prev_total_error[0][0] += pose_error[0][0]
        prev_total_error[1][0] += pose_error[1][0]
        prev_total_error[2][0] += pose_error[2][0]
        prev_total_error[3][0] += pose_error[3][0]

        prev_time = now
        i += 0.033

        #Setting the values to be published
        vel.linear.x = vel_x[0] + uav1_vel.linear.x
        vel.linear.y = vel_y[0] + uav1_vel.linear.y
        vel.linear.z = vel_z[0] + uav1_vel.linear.z
        pub2.publish(vel)

        vel.linear.x = vel_x[1] + uav1_vel.linear.x
        vel.linear.y = vel_y[1] + uav1_vel.linear.y
        vel.linear.z = vel_z[1] + uav1_vel.linear.z
        pub3.publish(vel)

        vel.linear.x = vel_x[2] + uav1_vel.linear.x
        vel.linear.y = vel_y[2] + uav1_vel.linear.y
        vel.linear.z = vel_z[2] + uav1_vel.linear.z
        pub4.publish(vel)

        vel.linear.x = vel_x[3] + uav1_vel.linear.x
        vel.linear.y = vel_y[3] + uav1_vel.linear.y
        vel.linear.z = vel_z[3] + uav1_vel.linear.z
        pub5.publish(vel)

        prev_pose_error = pose_error

        rate.sleep()

def follow2():
