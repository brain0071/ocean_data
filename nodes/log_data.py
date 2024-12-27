#!/usr/bin/env python

import rospy
from mavros_msgs.msg import ActuatorControl
from mavros_msgs.msg import WheelOdomStamped
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
import tf
import numpy as np
import pandas as pd
import time
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped
import csv

class LOG_Wrapper():
    def __init__(self):

        self.log_state = True
        
        rospy.Service("/stop_log", SetBool, self.log_state_callback)
    
        self.att = np.array([1., 0., 0., 0.])  # Quaternion format: qw, qx, qy, qz
        self.euler = np.zeros((3,))
        self.rate = np.zeros((3,))
        self.control = np.zeros((3,))
        self.motor = np.zeros((8,))
        
        self.update_time = time.time()
         
        # angular
        rospy.Subscriber("/filter/quaternion", QuaternionStamped, self.updateAtt_callback, queue_size=10)
        
        # angular velocity data
        rospy.Subscriber("/imu/angular_velocity", Vector3Stamped, self.rate_callback_imu, queue_size= 10)
        
        # control
        rospy.Subscriber("/manual_control", WheelOdomStamped, self.updateControl_callback, queue_size=10)
        
        # motor
        rospy.Subscriber("/mavros/actuator_control", ActuatorControl, self.motor_callback, queue_size= 10)
        
        rospy.spin()
    
    def motor_callback(self, msg):
        self.motor = msg.controls
        
    def rate_callback_imu(self, msg):
        self.rate = [msg.vector.x, msg.vector.y, msg.vector.z]
        
        # rospy.loginfo("rate: %f, %f, %f", self.rate[0], self.rate[1], self.rate[2])
        
    def updateAtt_callback(self, msg):
         
        # qw qx qy qz
        self.att = [msg.quaternion.w,  msg.quaternion.x,  msg.quaternion.y,  msg.quaternion.z]
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w]) 
        
        self.euler = [roll, pitch, yaw]

        # rospy.loginfo("att: %f, %f, %f, %f", self.att[0], self.att[1], self.att[2], self.att[3])
        
    
    def updateControl_callback(self, msg):

        if self.log_state == True:
            self.update_time = time.time() 
            self.control = [msg.data[3], msg.data[4], msg.data[5]]
            rospy.loginfo("control: %f, %f, %f", self.control[0], self.control[1], self.control[2])
            
            # Log data 
            new_row_data = {'time':self.update_time,     
                            'qw':self.att[0], 
                            'qx':self.att[1], 
                            'qy':self.att[2], 
                            'qz':self.att[3],

                            'phi':self.euler[0],
                            'theta': self.euler[1],
                            'psi': self.euler[2],    

                            'p':self.rate[0], 
                            'q':self.rate[1], 
                            'r':self.rate[2],
       
                            'u_4': self.control[0],
                            'u_5': self.control[1],
                            'u_6': self.control[2],
                            
                            'm_1': self.motor[0],
                            'm_2': self.motor[1],
                            'm_3': self.motor[2],
                            'm_4': self.motor[3],
                            'm_5': self.motor[4],
                            'm_6': self.motor[5],
                            'm_7': self.motor[6],
                            'm_8': self.motor[7],
                            
                            }
        
            with open('/home/dlmux/WorkSpace/data/Data_log.csv', 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=new_row_data.keys())
                writer.writerow(new_row_data)
            
        elif self.log_state == False:
            rospy.logerr("Stop log data...")
            
        else:
            rospy.logerr("Controller is not running...")
    
    def log_state_callback(self, msg):
        if msg.data == True:
            self.log_state = False
            return True
        else:
            self.log_state = True
            return False
    
def main():

    rospy.init_node("log_node")
    LOG_Wrapper()

if __name__ == '__main__':
    main()
