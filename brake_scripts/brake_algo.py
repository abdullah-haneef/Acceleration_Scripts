#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import datetime
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import *
from geometry_msgs.msg import *
import message_filters
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from std_msgs.msg import Float64MultiArray


global steer_list
steer_list = []

angle_range = 360 #input range of actual lidar
error = 0.0
forward_projection = 0.5		
desired_distance = 1.0
car_length = 0.3302   #meters
look_ahead = 3   #meters

pub2 = rospy.Publisher('/point',PointStamped,queue_size=1)   #publisher for a pointstamped topic to visualize lookahead point in rviz 

print("pure_pursuit controller started ")


def odometryCallback(self, odometry):
    self.carPosX = odometry.pose.pose.position.x
    self.carPosY = odometry.pose.pose.position.y
    quaternion = odometry.pose.pose.orientation
    (r, p, y) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    self.theta = y


# function to get the steering value for a given lookahead distance 
def get_steering(goal_y):
    steer= math.atan((2*goal_y*car_length)/(look_ahead**2))
    
    if steer > 50*np.pi/180:
        steer = 50*np.pi/180
    if steer < -50*np.pi/180:
        steer = -50*np.pi/180
    return steer 



# function to inject points in the path recieved from the planning algorithm 
def add_pts(list):
    for i in range(len(list)-1):
        x_mid= (list[i][0]+list[i+1][0])/2
        y_mid= (list[i][1]+list[i+1][1])/2
        list.append(list[i])
        list.append([round(x_mid,3),round(y_mid,3)])
    return list 




#  function to find a lookahead point on the path recieved from planning algorithm
def get_goal(path,look_dist):
    #t=path[0:10]
    for i in path:
        a = math.sqrt((i[0])**2 +(i[1])**2)
        if a < look_ahead-2 or a > look_ahead +2:
            pass
            # print(i,a)
        else :
            # print('final',a)
            x_goal=i[0]
            y_goal=i[1]
            break
    p_msg = PointStamped()
    p_msg.header.frame_id = 'base_link'
    p_msg.point.x = x_goal 
    p_msg.point.y = y_goal 
    pub2.publish(p_msg)
    
    return x_goal,y_goal




def drawing(img, detection_queue, data):
    detection_list = detection_queue
    detections, width_ratio, height_ratio = detection_list
    img_bbox = np.zeros(img.shape).astype(np.float32)
    list_detect = []
    img_bbox.fill(255)
    delta = 10

    for label, confidence, bbox in detections:

        left, top, right, bottom = bbox2points(bbox)
        left, top, right, bottom = int(left * width_ratio), int(top * height_ratio), int(right * width_ratio), int(
            bottom * height_ratio)

        if label.lower()=='yellow':
            cv2.rectangle(img, (left-delta, top-delta), (right+delta, bottom+delta), (0, 255, 0), -1)
        elif label.lower()=='blue':
            cv2.rectangle(img, (left-delta, top-delta), (right+delta, bottom+delta), (255, 0, 0), -1)
        elif label.lower()=='orange':
            cv2.rectangle(img, (left-delta, top-delta), (right+delta, bottom+delta), (0, 0, 255), -1)

    


def detections(img, data):
    detections, width_ratio, height_ratio = darknet_helper(img, width, height)
    detection_queue = [detections, width_ratio, height_ratio]
    drawing(img, detection_queue, data)
    return detection_queue


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.
    
	index = angle * (len(data.ranges)/360)
	return data.ranges[int(index)]


#  subscriber callback
def callback(odom,path):

    global steer_list
    carPosx=odom.pose.pose.position.x
    carPosy=odom.pose.pose.position.y
    quaternion = odom.pose.pose.orientation
    (r, p, yaw) = euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    theta = yaw

    way = [[carPosx,carPosy]] 
    temp=path.data
    
    for i in range(0, len(temp)-1, 2):
        way.append([temp[i], temp[i+1]])
    
    print('check_call')

    added_path = add_pts(way)
    new_path=[]

    for a,i in enumerate(added_path) :
    
        x=i[0]
        y=i[1]
        x_= x-carPosx
        y_= carPosy-y
        x_new = x_*math.cos(theta) - y_*math.sin(theta)
        y_new = -x_*math.sin(theta) - y_*math.cos(theta)
        if x_new > 0:
            new_path.append([x_new,y_new])

    try:
        goal_x,goal_y = get_goal(new_path,look_ahead)
        steer = get_steering(goal_y)
        vel = 14*((math.exp((steer*180/np.pi)/6))/(1+math.exp((steer*180/np.pi)/6))**2)
        if vel < 1:
            vel = 1.5
        print(vel)
        steer_list.append(steer)
    except :
        steer =steer_list[-1]
        vel = 1.5
        print("exception")

    msg = AckermannDriveStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = '/map'
    msg.drive.steering_angle = steer*2
    msg.drive.speed = vel  #m/s
    pub.publish(msg)

    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, 'bgr8')
    
    detection_list = detections(image, data)
    detections, width_ratio, height_ratio = detection_list
    label, confidence, bbox = detections

    current_time = datetime.datetime.now()
    x = datetime.datetime.now()
   
    time_difference = (datetime.datetime.now() - new_time).seconds
    current_vel = odom.twist.twist.linear.x
    old_vel = current_vel

    state = 2

    if label == 'orange':
        state = 1

    elif r>b and r>g and state == 2:
        _point = self.transform_point(point)
        orange_cone.append(_point)
        self.orange_cone.append(_point)

        vertical_dist = orange_cone[0]



def wall_follow_cb (data):
	global forward_projection
	global desired_distance 
	global error

	print(desired_distance)
	theta = 330 
	
	a = getRange(data,theta) 
	b = getRange(data, 270) 

	theta = math.radians(theta)

	alpha = math.atan((((a * (math.cos(theta))) - b) / (a * (math.sin(theta)))))
	
	ab = b * math.cos(alpha) 
	cd = ab + (forward_projection * math.sin(alpha))
	
	error = desired_distance - cd	
	
	msg = pid_input()
	msg.pid_error = error



def talker():
    pub = rospy.Publisher('braking_state', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node("deceleration_node")
    current_time = datetime.datetime.now()
    talker()
	rospy.Subscriber("/scan", LaserScan, wall_follow_cb)
    rospy.spin()