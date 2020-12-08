#! /usr/bin/env python
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
global uav6_pose
global uav7_pose
global uav8_pose
global uav9_pose
global quat
global uav1_vel
uav1_pose=PoseStamped()
uav2_pose=PoseStamped()
uav3_pose=PoseStamped()
uav4_pose=PoseStamped()
uav5_pose=PoseStamped()
uav6_pose=PoseStamped()
uav7_pose=PoseStamped()
uav8_pose=PoseStamped()
uav9_pose=PoseStamped()
quat = Quaternion()
uav1_vel = Twist()
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

def uav6PoseCallback(data):
    global uav6_pose
    uav6_pose = data

def uav7PoseCallback(data):
    global uav7_pose
    uav7_pose = data

def uav8PoseCallback(data):
    global uav8_pose
    uav8_pose = data

def uav9PoseCallback(data):
    global uav9_pose
    uav9_pose = data


def uav1VelCallback(data):
    global uav1_vel
    uav1_vel = data

rospy.init_node('Follower',anonymous=True)
rospy.Subscriber('/uav1/ground_truth_to_tf/pose',PoseStamped,uav1PoseCallback)
rospy.Subscriber('/uav2/ground_truth_to_tf/pose',PoseStamped,uav2PoseCallback)
rospy.Subscriber('/uav3/ground_truth_to_tf/pose',PoseStamped,uav3PoseCallback)
rospy.Subscriber('/uav4/ground_truth_to_tf/pose',PoseStamped,uav4PoseCallback)
rospy.Subscriber('/uav5/ground_truth_to_tf/pose',PoseStamped,uav5PoseCallback)
rospy.Subscriber('/uav6/ground_truth_to_tf/pose',PoseStamped,uav6PoseCallback)
rospy.Subscriber('/uav7/ground_truth_to_tf/pose',PoseStamped,uav7PoseCallback)
rospy.Subscriber('/uav8/ground_truth_to_tf/pose',PoseStamped,uav8PoseCallback)
rospy.Subscriber('/uav9/ground_truth_to_tf/pose',PoseStamped,uav9PoseCallback)
rospy.Subscriber('/uav1/cmd_vel',Twist,uav1VelCallback)

pub2 = rospy.Publisher('/uav2/cmd_vel',Twist,queue_size=10)
pub3 = rospy.Publisher('/uav3/cmd_vel',Twist,queue_size=10)
pub4 = rospy.Publisher('/uav4/cmd_vel',Twist,queue_size=10)
pub5 = rospy.Publisher('/uav5/cmd_vel',Twist,queue_size=10)
pub6 = rospy.Publisher('/uav6/cmd_vel',Twist,queue_size=10)
pub7 = rospy.Publisher('/uav7/cmd_vel',Twist,queue_size=10)
pub8 = rospy.Publisher('/uav8/cmd_vel',Twist,queue_size=10)
pub9 = rospy.Publisher('/uav9/cmd_vel',Twist,queue_size=10)

pid_x = [1.05,0.017,0.31]
pid_y = [1.05,0.017,0.31]
pid_z = [1.05,0.017,0.31]

vel = Twist()
rate = rospy.Rate(30)

pose_error = []
prev_pose_error = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
prev_total_error = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

uav2_pose_leader_frame = np.array([-2,0,0])
uav3_pose_leader_frame = np.array([2,0,0])
uav4_pose_leader_frame = np.array([0,2,0])
uav5_pose_leader_frame = np.array([0,-2,0])
uav6_pose_leader_frame = np.array([0,-4,0])
uav7_pose_leader_frame = np.array([0,4,0])
uav8_pose_leader_frame = np.array([-4,0,0])
uav9_pose_leader_frame = np.array([4,0,0])

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
    new_loc_uav6 = quat.rotate(uav6_pose_leader_frame)
    new_loc_uav7 = quat.rotate(uav7_pose_leader_frame)
    new_loc_uav8 = quat.rotate(uav8_pose_leader_frame)
    new_loc_uav9 = quat.rotate(uav9_pose_leader_frame)
    #set the location the ddrone has to reach in global frame
    uav2_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav2[0],uav1_pose.pose.position.y+new_loc_uav2[1],uav1_pose.pose.position.z+new_loc_uav2[2]]
    uav3_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav3[0],uav1_pose.pose.position.y+new_loc_uav3[1],uav1_pose.pose.position.z+new_loc_uav3[2]]
    uav4_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav4[0],uav1_pose.pose.position.y+new_loc_uav4[1],uav1_pose.pose.position.z+new_loc_uav4[2]]
    uav5_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav5[0],uav1_pose.pose.position.y+new_loc_uav5[1],uav1_pose.pose.position.z+new_loc_uav5[2]]
    uav6_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav6[0],uav1_pose.pose.position.y+new_loc_uav6[1],uav1_pose.pose.position.z+new_loc_uav6[2]]
    uav7_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav7[0],uav1_pose.pose.position.y+new_loc_uav7[1],uav1_pose.pose.position.z+new_loc_uav7[2]]
    uav8_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav8[0],uav1_pose.pose.position.y+new_loc_uav8[1],uav1_pose.pose.position.z+new_loc_uav8[2]]
    uav9_pose_global_frame = [uav1_pose.pose.position.x+new_loc_uav9[0],uav1_pose.pose.position.y+new_loc_uav9[1],uav1_pose.pose.position.z+new_loc_uav9[2]]


    #print(uav2_pose_global_frame)

    #Calculate errors
    pose_error = [[uav2_pose_global_frame[0]-uav2_pose.pose.position.x,uav2_pose_global_frame[1]-uav2_pose.pose.position.y,uav2_pose_global_frame[2]-uav2_pose.pose.position.z]
    ,[uav3_pose_global_frame[0]-uav3_pose.pose.position.x,uav3_pose_global_frame[1]-uav3_pose.pose.position.y,uav3_pose_global_frame[2]-uav3_pose.pose.position.z]
    ,[uav4_pose_global_frame[0]-uav4_pose.pose.position.x,uav4_pose_global_frame[1]-uav4_pose.pose.position.y,uav4_pose_global_frame[2]-uav4_pose.pose.position.z]
    ,[uav5_pose_global_frame[0]-uav5_pose.pose.position.x,uav5_pose_global_frame[1]-uav5_pose.pose.position.y,uav5_pose_global_frame[2]-uav5_pose.pose.position.z]
    ,[uav6_pose_global_frame[0]-uav6_pose.pose.position.x,uav6_pose_global_frame[1]-uav6_pose.pose.position.y,uav6_pose_global_frame[2]-uav6_pose.pose.position.z]
    ,[uav7_pose_global_frame[0]-uav7_pose.pose.position.x,uav7_pose_global_frame[1]-uav7_pose.pose.position.y,uav7_pose_global_frame[2]-uav7_pose.pose.position.z]
    ,[uav8_pose_global_frame[0]-uav8_pose.pose.position.x,uav8_pose_global_frame[1]-uav8_pose.pose.position.y,uav8_pose_global_frame[2]-uav8_pose.pose.position.z]
    ,[uav9_pose_global_frame[0]-uav9_pose.pose.position.x,uav9_pose_global_frame[1]-uav9_pose.pose.position.y,uav9_pose_global_frame[2]-uav9_pose.pose.position.z]]
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
    if(pose_error[4][0]<=0.005):
        prev_total_error[4][0] = 0
    if(pose_error[5][0]<=0.005):
        prev_total_error[5][0] = 0
    if(pose_error[6][0]<=0.005):
        prev_total_error[6][0] = 0
    if(pose_error[7][0]<=0.005):
        prev_total_error[7][0] = 0


    if(pose_error[0][1]<=0.005):
        prev_total_error[0][1] = 0
    if(pose_error[1][1]<=0.005):
        prev_total_error[1][1] = 0
    if(pose_error[2][1]<=0.005):
        prev_total_error[2][1] = 0
    if(pose_error[3][1]<=0.005):
        prev_total_error[3][1] = 0
    if(pose_error[4][1]<=0.005):
        prev_total_error[4][1] = 0
    if(pose_error[5][1]<=0.005):
        prev_total_error[5][1] = 0
    if(pose_error[6][1]<=0.005):
        prev_total_error[6][1] = 0
    if(pose_error[7][1]<=0.005):
        prev_total_error[7][1] = 0

    if(pose_error[0][2]<=0.005):
        prev_total_error[0][2] = 0
    if(pose_error[1][2]<=0.005):
        prev_total_error[1][2] = 0
    if(pose_error[2][2]<=0.005):
        prev_total_error[2][2] = 0
    if(pose_error[3][2]<=0.005):
        prev_total_error[3][2] = 0
    if(pose_error[4][2]<=0.005):
        prev_total_error[4][2] = 0
    if(pose_error[5][2]<=0.005):
        prev_total_error[5][2] = 0
    if(pose_error[6][2]<=0.005):
        prev_total_error[6][2] = 0
    if(pose_error[7][2]<=0.005):
        prev_total_error[7][2] = 0

    #Calculating the pid values
    vel_x = [pid_x[0]*pose_error[0][0]+pid_x[1]*((prev_pose_error[0][0]-pose_error[0][0])/diff) + pid_x[1]*prev_total_error[0][0]
    ,pid_x[0]*pose_error[1][0]+pid_x[1]*((prev_pose_error[1][0]-pose_error[1][0])/diff) + pid_x[1]*prev_total_error[1][0]
    ,pid_x[0]*pose_error[2][0]+pid_x[1]*((prev_pose_error[2][0]-pose_error[2][0])/diff) + pid_x[1]*prev_total_error[2][0]
    ,pid_x[0]*pose_error[3][0]+pid_x[1]*((prev_pose_error[3][0]-pose_error[3][0])/diff) + pid_x[1]*prev_total_error[3][0]
    ,pid_x[0]*pose_error[4][0]+pid_x[1]*((prev_pose_error[4][0]-pose_error[4][0])/diff) + pid_x[1]*prev_total_error[4][0]
    ,pid_x[0]*pose_error[5][0]+pid_x[1]*((prev_pose_error[5][0]-pose_error[5][0])/diff) + pid_x[1]*prev_total_error[5][0]
    ,pid_x[0]*pose_error[6][0]+pid_x[1]*((prev_pose_error[6][0]-pose_error[6][0])/diff) + pid_x[1]*prev_total_error[6][0]
    ,pid_x[0]*pose_error[7][0]+pid_x[1]*((prev_pose_error[7][0]-pose_error[7][0])/diff) + pid_x[1]*prev_total_error[7][0]]

    vel_y = [pid_y[0]*pose_error[0][1]+pid_y[1]*((prev_pose_error[0][1]-pose_error[0][1])/diff) + pid_y[1]*prev_total_error[0][1]
    ,pid_y[0]*pose_error[1][1]+pid_y[1]*((prev_pose_error[1][1]-pose_error[1][1])/diff) + pid_y[1]*prev_total_error[1][1]
    ,pid_y[0]*pose_error[2][1]+pid_y[1]*((prev_pose_error[2][1]-pose_error[2][1])/diff) + pid_y[1]*prev_total_error[2][1]
    ,pid_y[0]*pose_error[3][1]+pid_y[1]*((prev_pose_error[3][1]-pose_error[3][1])/diff) + pid_y[1]*prev_total_error[3][1]
    ,pid_y[0]*pose_error[4][1]+pid_y[1]*((prev_pose_error[4][1]-pose_error[4][1])/diff) + pid_y[1]*prev_total_error[4][1]
    ,pid_y[0]*pose_error[5][1]+pid_y[1]*((prev_pose_error[5][1]-pose_error[5][1])/diff) + pid_y[1]*prev_total_error[5][1]
    ,pid_y[0]*pose_error[6][1]+pid_y[1]*((prev_pose_error[6][1]-pose_error[6][1])/diff) + pid_y[1]*prev_total_error[6][1]
    ,pid_y[0]*pose_error[7][1]+pid_y[1]*((prev_pose_error[7][1]-pose_error[7][1])/diff) + pid_y[1]*prev_total_error[7][1]]

    vel_z = [pid_z[0]*pose_error[0][2]+pid_z[2]*((prev_pose_error[0][2]-pose_error[0][2])/diff) + pid_z[1]*prev_total_error[0][2]
    ,pid_z[0]*pose_error[1][2]+pid_z[2]*((prev_pose_error[1][2]-pose_error[1][2])/diff) + pid_z[1]*prev_total_error[1][2]
    ,pid_z[0]*pose_error[2][2]+pid_z[2]*((prev_pose_error[2][2]-pose_error[2][2])/diff) + pid_z[1]*prev_total_error[2][2]
    ,pid_z[0]*pose_error[3][2]+pid_z[2]*((prev_pose_error[3][2]-pose_error[3][2])/diff) + pid_z[1]*prev_total_error[3][2]
    ,pid_z[0]*pose_error[4][2]+pid_z[2]*((prev_pose_error[4][2]-pose_error[4][2])/diff) + pid_z[1]*prev_total_error[4][2]
    ,pid_z[0]*pose_error[5][2]+pid_z[2]*((prev_pose_error[5][2]-pose_error[5][2])/diff) + pid_z[1]*prev_total_error[5][2]
    ,pid_z[0]*pose_error[6][2]+pid_z[2]*((prev_pose_error[6][2]-pose_error[6][2])/diff) + pid_z[1]*prev_total_error[6][2]
    ,pid_z[0]*pose_error[7][2]+pid_z[2]*((prev_pose_error[7][2]-pose_error[7][2])/diff) + pid_z[1]*prev_total_error[7][2]]

    print('Working')

    #Update the errors for the integrals
    prev_total_error[0][2] += pose_error[0][2]
    prev_total_error[1][2] += pose_error[1][2]
    prev_total_error[2][2] += pose_error[2][2]
    prev_total_error[3][2] += pose_error[3][2]
    prev_total_error[4][2] += pose_error[4][2]
    prev_total_error[5][2] += pose_error[5][2]
    prev_total_error[6][2] += pose_error[6][2]
    prev_total_error[7][2] += pose_error[7][2]

    prev_total_error[0][1] += pose_error[0][1]
    prev_total_error[1][1] += pose_error[1][1]
    prev_total_error[2][1] += pose_error[2][1]
    prev_total_error[3][1] += pose_error[3][1]
    prev_total_error[4][1] += pose_error[4][1]
    prev_total_error[5][1] += pose_error[5][1]
    prev_total_error[6][1] += pose_error[6][1]
    prev_total_error[7][1] += pose_error[7][1]

    prev_total_error[0][0] += pose_error[0][0]
    prev_total_error[1][0] += pose_error[1][0]
    prev_total_error[2][0] += pose_error[2][0]
    prev_total_error[3][0] += pose_error[3][0]
    prev_total_error[4][0] += pose_error[4][0]
    prev_total_error[5][0] += pose_error[5][0]
    prev_total_error[6][0] += pose_error[6][0]
    prev_total_error[7][0] += pose_error[7][0]

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

    vel.linear.x = vel_x[4] + uav1_vel.linear.x
    vel.linear.y = vel_y[4] + uav1_vel.linear.y
    vel.linear.z = vel_z[4] + uav1_vel.linear.z
    pub6.publish(vel)

    vel.linear.x = vel_x[5] + uav1_vel.linear.x
    vel.linear.y = vel_y[5] + uav1_vel.linear.y
    vel.linear.z = vel_z[5] + uav1_vel.linear.z
    pub7.publish(vel)

    vel.linear.x = vel_x[6] + uav1_vel.linear.x
    vel.linear.y = vel_y[6] + uav1_vel.linear.y
    vel.linear.z = vel_z[6] + uav1_vel.linear.z
    pub8.publish(vel)

    vel.linear.x = vel_x[7] + uav1_vel.linear.x
    vel.linear.y = vel_y[7] + uav1_vel.linear.y
    vel.linear.z = vel_z[7] + uav1_vel.linear.z
    pub9.publish(vel)

    prev_pose_error = pose_error

    rate.sleep()
