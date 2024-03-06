#!/usr/bin/env python3 

import rospy 
import math 
import tf 
import numpy

from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Dock_at_Tag:

    def __init__(self):
        rospy.init_node("dock_at_art_tag", anonymous=False)
        rospy.loginfo("Welcome, This node Docks the robot at AR Tag ID ----> 0")
        
        self.p_distance = 1
        self.i_distance = 0.01
        self.d_distance = 0.5

        self.p_angle = 1
        self.i_angle = 0.03
        self.d_angle = 0.05

        self.controlled_distance = 0.0
        self.controlled_angle = 0.0

        self.path_angle = 0.0
        self.diff_angle = 0.0

        self.base_frame = 'base_footprint'
        self.odom_frame = 'odom'

        self.tf_listener = tf.TransformListener()

        self.rotation = 0.0
        self.prev_rotation = 0.0
        self.previous_angle = 0.0
        self.previous_distance = 0.0
        self.total_distance = 0.0

        self.distance = 0.0
        self.diff_distance = 0.0

        self.x_tag = 2.0
        self.y_tag = 5.0
        self.angle_z_tag = numpy.deg2rad(50)

        self.x_robot = 0.0
        self.y_robot = 0.0
        self.angle_z_robot = 0.0

        self.pub_speed = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def compute(self):
        velocity = Twist()
        self.distance = math.sqrt(math.pow((self.x_tag - self.x_robot), 2)+math.pow(self.y_tag-self.y_robot, 2))

        while self.distance > 0.05:
            self.path_angle = math.atan2(self.y_tag - self.y_robot, self.x_tag - self.x_robot)
            if self.path_angle < -math.pi/4 or self.path_angle > math.pi/4:
                if self.y_tag < 0 and self.y_robot < self.y_tag:
                    self.path_angle = -2*math.pi + self.path_angle
                elif self.y_tag >=0 and self.y_robot > self.y_tag:
                    self.path_angle = 2 * math.pi + self.path_angle
            if self.prev_rotation > math.pi-0.1 and self.rotation <=0:
                self.rotation = 2* math.pi + self.rotation
            elif self.prev_rotation < -math.pi+0.1 and self.rotation > 0:
                self.rotation = -2*math.pi + self.rotation
            
        
            self.diff_angle = self.path_angle - self.previous_angle
            self.diff_distance = self.distance - self.previous_distance

            
            
            self.controlled_distance = self.p_distance * self.distance + self.i_distance * self.distance + self.d_distance * self.distance
            self.controlled_angle = self.p_angle * self.path_angle + self.i_angle * self.path_angle + self.d_angle * self.path_angle
            velocity.angular.z = self.controlled_angle - self.rotation
            velocity.linear.x = self.controlled_distance - self.rotation
            
            if velocity.angular.z > 0:
                velocity.angular.z = min(velocity.angular.z, 1.5)
            else:
                velocity.angular.z = max(velocity.angular.z, -1.5)
            
            print(self.controlled_angle, self.controlled_distance)
            self.pub_speed.publish(velocity)
        self.prev_rotation = self.rotation
        self.previous_angle = self.path_angle
        self.previous_distance = self.distance
        
        print(self.controlled_angle, self.controlled_distance)
    
    def ros_spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.compute()
            r.sleep()

    def odom_callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        self.angle_z_robot = msg.pose.pose.orientation.z

if __name__=='__main__':
    
    dock_at_tag = Dock_at_Tag()
    try:
        dock_at_tag.ros_spin()
    except rospy.ROSInternalException:
        pass
        
    