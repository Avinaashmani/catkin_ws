#!/usr/bin/env python3

import rospy
import numpy as np
import tf

from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point


kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

class GotoTag():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        position = Point()
        self.tag_posi = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)
        
        self.tf_listener_robot = tf.TransformListener()
        self.tf_listener_tag = tf.TransformListener()
        self.odom_frame = 'odom'

        self.parent_frame_tag = "camera_rgb_optical_frame"
        self.child_frame_tag = "tag_0"

        try:
            self.tf_listener_robot.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener_robot.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        
        (position, rotation) = self.get_odom()

        last_rotation = 0
        
        #tag_posi= type(self.get_tag())
        
        self.initial_x = 0
        self.initial_y = 0.0
        self.initial_angle_z = 0.0

        rospy.Timer(rospy.Duration(0.1), self.get_tag())

        x__ = self.initial_x

        (goal_x, goal_y, goal_z) = 5*(self.initial_x), self.initial_y, self.initial_angle_z
        
        if goal_z > 180 or goal_z < -180:
            print("you input wrong z range.")
            self.shutdown()
        #goal_z = np.deg2rad(goal_z)
 
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        #distance is the error for length, x,y
        distance = goal_distance
        previous_distance = 0
        total_distance = 0

        previous_angle = 0
        total_angle = 0


        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            #path_angle = error
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation



            diff_angle = path_angle - previous_angle
            diff_distance = distance - previous_distance

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))

            control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

            control_signal_angle = kp_angle*path_angle + ki_angle*total_angle + kd_distance*diff_angle

            move_cmd.angular.z = (control_signal_angle) - rotation
            move_cmd.linear.x = min(control_signal_distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            previous_distance = distance
            total_distance = total_distance + distance
            print("Current position and rotation are: ", (position, rotation))
            print(distance)

        (position, rotation) = self.get_odom()
        #print("Current positin and rotation are: ", (position, rotation))

        while abs(rotation - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if goal_z >= 0:
                if rotation <= goal_z and rotation >= goal_z - pi:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
            else:
                if rotation <= goal_z + pi and rotation > goal_z:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = -0.5
                else:
                    move_cmd.linear.x = 0.00
                    move_cmd.angular.z = 0.5
                pass
            self.cmd_vel.publish(move_cmd)
            r.sleep()

        #print(self.initial_x)
        #print(self.initial_y)
        #rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.signal_shutdown(self.shutdown())
        return

    def getkey(self):
        global x_input, y_input, z_input
        x = x_input
        y = y_input
        z = z_input
        if x == 's':
            self.shutdown()
        x, y, z = [float(x), float(y), float(z)]
        return x, y, z

    def get_odom(self):
        
        try:
            (trans, rot) = self.tf_listener_robot.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])
    
    def get_tag(self):
        
        try:
     
            (transalate, rotate) = self.tf_listener_tag.lookupTransform(self.parent_frame_tag, self.child_frame_tag, rospy.Time(0))

            rotation = euler_from_quaternion (rotate)
        
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        self.initial_x = transalate[2]

            

    def shutdown(self):
        cmd_vel_shutdown = Twist()
        cmd_vel_shutdown.linear.x = 0.0
        cmd_vel_shutdown.angular.z = 0.0
        self.cmd_vel.publish(cmd_vel_shutdown)
        rospy.sleep(1)


while not rospy.is_shutdown():
    try:
        GotoTag()
    except rospy.ROSInterruptException:
        rospy.signal_shutdown("Shutting down")
