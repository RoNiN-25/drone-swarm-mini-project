#! /usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import PoseStamped #Message type for psoe information
from geometry_msgs.msg import Twist #Message type for velocity information

global uav1_pose #global value for drone position
global prev_time #global value for storing the time

ns = rospy.get_namespace()

with open(sys.argv[1] + ns[:-1] + '_points.txt', 'r') as fp:
    out = fp.read()
    out = out.split(';')
    coords = [map(int, i.split()) for i in out[:-1]]
    path = map(int, out[-1].split())

print(coords, path)

uav1_pose = PoseStamped() #empty initialization

#callback for the position data
def uav1PoseCallback(data):
    global uav1_pose
    uav1_pose = data

# Ros node initialization
rospy.init_node('waypoints_traveller', anonymous=True)
# Subscribe to the topic publishing position
rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, uav1PoseCallback)
# Create a publisher for the velocities
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# PID values for the x,y and z axis. Change for better performance
pid_x = [0.5,0.001,0.01]
pid_y = [0.5,0.001,0.01]
pid_z = [0.5,0.001,0.01]

# variable to store the velocity to be published to the drone
vel = Twist()
# Rate at which to publish
rate = rospy.Rate(30)

# the errors to be used for PID
pose_error = [0, 0, 0]
prev_pose_error = [0, 0, 0]
prev_total_error = [0, 0, 0]

# destination position - (x,y,z)
dest_pose = coords[path[0]]

# the current time as caluclated bt ROS -> Time from the start of the ros master
prev_time = rospy.get_rostime().nsecs # get in nano seconds

# loop that runs as long as the ros master runs and there exists points to discover
while not rospy.is_shutdown() and path:

    if abs(pose_error[0]) < .03 and abs(pose_error[1]) < .03 and abs(pose_error[2]) < .03:
        path = path[1:]
        if path:
            dest_pose = coords[path[0]]
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
    if(pose_error[0]<=0.005):
        prev_total_error[0] = 0

    if(pose_error[1]<=0.005):
        prev_total_error[1] = 0

    if(pose_error[2]<=0.005):
        prev_total_error[2] = 0

    # Calculating the pid values
    # o/p = Kp*error + Ki*sum of errors + kd*(prev_err*curr_err)/time
    vel_x = pid_x[0]*pose_error[0] + pid_x[2]*((prev_pose_error[0]-pose_error[0])/diff) + pid_x[1]*prev_total_error[0]
    vel_y = pid_y[0]*pose_error[1] + pid_y[2]*((prev_pose_error[1]-pose_error[1])/diff) + pid_y[1]*prev_total_error[1]
    vel_z = pid_z[0]*pose_error[2] + pid_z[2]*((prev_pose_error[2]-pose_error[2])/diff) + pid_z[1]*prev_total_error[2]

    # Update the errors for the integrals
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
    vel.linear.x = vel_x
    vel.linear.y = vel_y
    vel.linear.z = vel_z
    
    #publish the data
    pub.publish(vel)
    
    #Set prev_error as the current_error to use in the next loop
    prev_pose_error = pose_error
    
    # Sleep for the rate time
    rate.sleep()

