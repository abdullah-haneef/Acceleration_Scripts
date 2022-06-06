#!/usr/bin/env python3
from time import sleep
import rospy
from sensor_msgs.msg import LaserScan 
from vehicle_msgs import AccelerationInstructions
from ackermann_msgs.msg import AckermannDrive
import math

angle = 0.00
pid = 0.00
kp = 0.00
kd = 0.00
error = 0.00
max_velocity = 100
state = 0
imu_data = 0.00
brake_actuator = 0.00 


# *max steering angle  = 10 to -10 degrees*
# *max acceleration  =  x*
# *max deceleration  =  x()
# state 0 is the state when the car is just placed on the track and is in ready to drive state
  
def receive_state_cb(data):
   global state  
   global error
   state = data.state
   error = data.error


def state_getter():
    rospy.init_node('state_receiver', anonymous = True)
    rospy.Subscriber("/acceleration_data", AccelerationInstructions, receive_state_cb)
    rospy.spin()


def acceleration():
    pub = rospy.Publisher('/cmd_vel', AckermannDrive, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        global max_velocity
        global state
        
        velocity_direction = AckermannDrive()
        
        if state == 1:
            velocity_direction.linear.x = max_velocity
            
            global pid
            global prev_error
            global kp
            global kd
            global angle
            
            
            print((float(error) - float(prev_error)))
            pid = (float(kp) * float(error)) + (float(kd) * (float(error) - float(prev_error)))  
            prev_error = error
            angle = -pid

        if state == 2:
            global brake_actuator
            global pid
            global prev_error
            global kp
            global kd
            global angle
            
            
            print((float(error) - float(prev_error)))
            pid = (float(kp) * float(error)) + (float(kd) * (float(error) - float(prev_error)))  
            prev_error = error
            angle = -pid
            
            velocity_direction.linear.x  = 0.00
            brake_actuator =  30
            while brake_actuator <80:
                brake_actuator +=5
                sleep(0.2)
        
        rospy.loginfo(velocity_direction)
        pub.publish(velocity_direction)
        rate.sleep()


if __name__ == '__main__':
    receive_state_cb()
    acceleration()













