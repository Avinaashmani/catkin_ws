#!/usr/bin/env python3

import rospy 
import math
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
from tf.transformations import euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray

class AprilTag_detection:

    def __init__(self):

        rospy.init_node("tag_detect_orient")
        rospy.loginfo("Welcome, Node has begun")

        self.rate = 10.0
        self.Q_size = 10.0

        self.odom_frame = 'odom'
        self.base_frame = 'base_foortprint'

        self.robot_orientation = 0.0
        self.robot_position = 0.0

        self.tag_position = 0.0
        self.tag_orientation = 0.0

        self.difference = 0.0

        self.trans = 0.0
        self.rot = 0.0
        self.rotation = 0.0

        self.kp_angle = 1
        self.ki_angle = 0.03
        self.kd_angle = 0.05

        self.total_angle = 0.0
        self.path_angle = 0.0
        self.diff_angle = 0.0

        self.current_angle = 0.0
        self.previous_angle = 0.0

        self.cmd_vel = Twist()

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/odom', Odometry, self.robot_callback)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        self.pub = rospy.Publisher ('/cmd_vel', Twist, queue_size=self.Q_size)

    def compute(self):
        
        self.path_angle = math.atan2(self.tag_orientation - self.robot_orientation)
        self.diff_angle = self.path_angle - self.previous_angle
        
        control_signal_angle = self.kp_angle*self.path_angle + self.ki_angle*self.total_angle + self.kd_angle*self.diff_angle
        print(control_signal_angle - self.rotation)
        
        self.cmd_vel.angular.z = self.difference
        self.pub.publish(self.cmd_vel)

        self.previous_angle = self.path_angle
    
    def ros_spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.compute()
            r.sleep()
    
    def tag_callback(self, msg):
        self.tag_orientation = msg.detections[0].pose.pose.pose.orientation.y
    
    def robot_callback(self,msg):
        self.robot_orientation = msg.pose.pose.orientation.z

    def get_rot(self):

        try:
            (self.trans, self.rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            self.rotation = euler_from_quaternion(self.rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*self.trans), self.rotation[2])
    
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

if __name__=='__main__':

    ar_tag_detect = AprilTag_detection()
    try:
        ar_tag_detect.ros_spin()
    except rospy.ROSInterruptException:
        pass