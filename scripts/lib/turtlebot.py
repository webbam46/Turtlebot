#!/usr/bin/env python
import rospy
from time import sleep
from geometry_msgs.msg import Twist
from lib.turtlebot_sensors import *
from std_msgs.msg import *
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
import math
import time


#Low level control class
class Control:
    def __init__(self,bot,wheel_radius,robot_radius):
        rospy.loginfo("Creating low level controller")
        #The turtlebot using the controller        
        self.bot = bot
        #Robot wheel radius
        self.wheel_radius = wheel_radius
        #Robot radius
        self.robot_radius = robot_radius
        #w_l sub
        rospy.loginfo("Creating wheel velocity subscriber")
        self.wheel_vals_sub = rospy.Subscriber("/wheel_values",Float32MultiArray,self._callback)
        self.w_l = 0
        self.w_r = 0
        rospy.loginfo("Controller created!")
    #Callback function
    def _callback(self,data):
        print "callback working"
        #Get data
        if(data!=None):
            _data = data.data
            if(len(_data)==2):
                rospy.loginfo("Recieved wheel velocities " + str(_data))
                w_l = _data[0]
                w_r = _data[1]
                rospy.loginfo("Left: " + str(w_l))
                rospy.loginfo("Right: " + str(w_r))
                self.w_l = w_l
                self.w_r = w_r
                #Compute forward kinematics               
                v,a = self.ForwardKinematics(self.w_l,self.w_r)
                #Now try and publish a twist message
                if(self.bot!=None):
                    self.bot.Publish(float(v),float(a))
                
            else:
                rospy.logerr("Invalid array size")
        else:
            rospy.logerr("Controller data is NULL")
    #Compute forward kinematics for differential drive
    def ForwardKinematics(self,w_l,w_r):
        c_l = self.wheel_radius * w_l
        c_r = self.wheel_radius * w_r
        v  = (c_l + c_r) /2
        a = (c_l - c_r) / self.wheel_radius
        rospy.loginfo("Forward kinematics " + str(v) + " " +  str(a))
        return (v,a)
    def PublishForwardKinematics(self,w_l,w_r):
        v,a = self.ForwardKinematics(w_l,w_r)
        self.bot.Publish(self.bot.linear_speed,a)
    #compute inverse kinematics for differential drive
    def InverseKinematics(self,v,a):
        c_l = v + (self.robot_radius * a) / 2
        c_r = v - (self.robot_radius * a) / 2
        w_l = c_l / self.wheel_radius
        w_r = c_r / self.wheel_radius
        rospy.loginfo("Inverse kinematics")
        rospy.loginfo("Wheel left: " + str(w_l))
        rospy.loginfo("Wheel right: " + str(w_r))
        return (w_l,w_r)
#
# Turtlebot class
#
class TurtleBot:
    #Specifications
    #Robot radius
    ROBOT_RADIUS = 0.15 #150mm 
    #ROBOT_RADIUS = 0.177 #177mm
    #Wheel radius
    WHEEL_RADIUS = 0.04
    #WHEEL_RADIUS = 0.35 #35mm
    
    #Initialise
    def __init__(self):
        #Initialise rospy        
        rospy.init_node('turtlebot')
        #cmd_vel publisher
        self.cmd_vel_publisher = rospy.Publisher('/turtlebot_1/cmd_vel',Twist);
        
        #Odom current        
        self.odom_current_x = 0.0
        self.odom_current_rot_z = 0.0
        #Odom callback
        def odom_callback(data):
            pose_x = data.pose.pose.position.x
            pos_rot_z = data.pose.pose.orientation.z
            self.odom_current_x = float(pose_x)
            self.odom_current_rot_z = float(pos_rot_z)
        
        #odometry callback        
        self.odom_sub = rospy.Subscriber('/turtlebot_1/odom',Odometry,odom_callback)
        #Linear speed
        self.linear_speed = 0.2
        #Angular speed
        self.angular_speed = 0.2
        #Variable for interacting with the turtlebots kinect sensor
        self.kinect = Kinect()
        #Movement checks        
        self.current_forward = 0
        self.current_rot = 0
        self.controller = self.CreateController()
        
        self.new_cmd = False
        #Shutdown callback function
        def shutdown_callback():
            #Stop the robot
            self.Stop()
            #Shutdown the kinect
            self.kinect.Shutdown()
        #
        # Setup callback if shutdown is requested
        #
        rospy.on_shutdown(shutdown_callback)
        #State initialised
        rospy.loginfo("TURTLEBOT INITIALISED")
        rospy.loginfo("Linear Velocity: " + str(self.linear_speed))
        rospy.loginfo("Angular Velocity: " + str(self.angular_speed))
    #Create low level wheel velocities controller
    def CreateController(self):
        return Control(self,TurtleBot.WHEEL_RADIUS,TurtleBot.ROBOT_RADIUS)
    #Publish cmd_vel data    
    def Publish(self,lx,az):
        #Publish velocities
        twist = Twist()
        #Set data
        twist.linear.x = lx
        twist.angular.z = az
        
        #Publish
        self.cmd_vel_publisher.publish(twist)
    #Stop the robot
    def Stop(self):
        twist = Twist()
        self.current_forward = 0
        self.current_rot = 0
        #Publish
        self.cmd_vel_publisher.publish(twist)
    #Move the robot forward
    def MoveForward(self):
        #rospy.logwarn("Moving robot forward with linear speed: " + str(self.linear_speed) + "m/s")
        self.Publish(self.linear_speed,0)
        self.current_forward = self.linear_speed
    def _MoveForward(self,dist):
        #rospy.loginfo("Distance: " + str(dist))
        self.MoveForward()
        sleep(dist / self.linear_speed)
        #When finished - stop the robot
        self.Stop()
    #Move the robot back
    def MoveBack(self):
        #rospy.logwarn("Moving robot back with linear speed: " + str(self.linear_speed) + "m/s")
        self.Publish(-self.linear_speed,0)
        self.current_forward = -self.linear_speed
    def _MoveBack(self,dist):
        #rospy.loginfo("Distance: " + str(dist) + "m")
        self.MoveBack()
        sleep(dist / self.linear_speed)
        #When finisihed - stop the robot
        self.Stop()
    #Rotate the robot left
    def RotateLeft(self):
        #rospy.logwarn("Rotating robot left with angular speed: " + str(self.angular_speed) + "m/s")
        self.Publish(self.current_forward,self.angular_speed)
        self.current_rot = self.angular_speed
    
    
    #def _RotateLeft(self,dist):
        #rospy.loginfo("Distance: " + str(dist) + "m")
   #     self.RotateLeft()
   #     sleep(dist / self.angular_speed)
        #When finished - stop the robot
   #     self.Stop()
    def _RotateLeft(self,dist):
        rad = math.radians(dist)
        current = float(self.odom_current_rot_z)
        target = float(current+rad)
        rospy.loginfo("Current odom: " + str(current))
        rospy.loginfo("Target: " + str(target))
        rospy.loginfo("rad: " + str(rad))
        while(self.odom_current_rot_z<rad):
            self.Publish(self.current_forward,self.angular_speed)
        self.Publish(self.current_forward,0)
        
    #Rotate the robot right
    def RotateRight(self):
        #rospy.logwarn("Rotating robot right with angular speed: " + str(self.angular_speed) + "m/s")
        self.Publish(self.current_forward,-self.angular_speed)
        self.current_rot = self.angular_speed
    def _RotateRight(self,dist):
        #rospy.loginfo("Distance: " + str(dist) + "m")
        self.RotateRight()
        sleep(dist / self.angular_speed)
        #When finished - stop the robot
        self.Stop()        
        
    #Track object
    #Radians to object is given
    def Track(self,dist,lindir,threshold):
        self.new_cmd = True
        self.new_cmd = False
        frac,whole = math.modf(float(dist))
        rospy.loginfo("current: " + str(whole))
        rospy.loginfo("frac: " + str(frac))
        if(frac>float(threshold)):
            if(lindir==1):
                self.MoveForward()
            elif(lindir==-1):
                self.MoveBack()
            if(whole==0):
                rospy.loginfo("Rotate left so object is in view")
                self.RotateLeft()
            elif(whole==1):
                rospy.loginfo("Rotate right so object is in view")
                self.RotateRight()
        else:
            rospy.loginfo("Object in view, just move forward")
            
    #Robot is running
    def Running(self):
        #Return if rospy is shutdown
        return not rospy.is_shutdown()