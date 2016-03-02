#!/usr/bin/env python
import rospy
import time
import thread
from lib.turtlebot import TurtleBot
from cv2 import *

#Braitenberg vehicle class
class BVehicle:
    #Initialise
    def __init__(self):
        #Turtlebot
        self.bot = TurtleBot()
        self.controller = self.bot.CreateController()
        #Need a quick angular speed to follow objects        
        self.bot.angular_speed = 0.1
        self.bot.linear_speed = 0.1
        #current Vehicle type        
        self.type = "love"
        #Target colour        
        self.target = "lightgreen"
        #Vehicle types
        self.types = [["love",self.Love],["fear",self.Fear]]
        #Log
        rospy.loginfo("Initialised braitenberg vehicle")
        #Start the menu thread
        #thread.start_new_thread(self.Menu,("menu",0))
    #Vehicle menu
    def Menu(self,thread,delay):
        while True:
            #Display a menu for changing the vehicle type
            print("Braitenberg Vehicle")
            for i in range(0,len(self.types)):
                _name = self.types[i][0]
                print(_name + " - " + _name.upper())
            print("Choose vehicle type - E.G love/fear")
            self.type = str(raw_input("Type option: "))
    def _callback(arg):
        pass
    #Main run method
    def Run(self):
        #Run depending on vehicle type
        for i in range(0,len(self.types)):
            _name = self.types[i][0]
            if(self.type==str(_name)):
                _func = self.types[i][1]
                _func()
    # *****************
    # LOVE
    # *****************
    def Love(self):
        rospy.loginfo("love")
        #Track object
        radians = self.bot.kinect.trackObject(self.target)
        #Object detected
        if(radians>0):
            #Set angular speed
            self.bot.angular_speed = 0.5
            #Move forward
            self.bot.MoveForward()
            #Rotate towards object
            self.bot.Track(radians,1,0.001)
        #Else too close to object
        elif(radians==-1):
            #Stop robot
            self.bot.Stop()
        #Else object not found
        else:
            #Increase angular speed
            self.bot.angular_speed = 1
            self.bot.Stop()
            #Rotate to find object
            self.bot.RotateLeft()
    # *****************
    # FEAR
    # *****************
    def Fear(self):
        #Track object
        radians = self.bot.kinect.trackObject(self.target)
        #Negate radians
        if(radians>1):
            radians = radians-1
        else:
            radians = radians + 1
        #Object detected
        if(radians>0):
            #Set angular speed
            self.bot.angular_speed = 0.5
            #Rotate depending on radians - should rotate away from object
            self.bot.Track(radians,-1,0.001)
        #Else too cose to object
        elif(radians==-1):
            #Stop the robot
            self.bot.Stop()
        #Else object not found
        else:
            #Set angular speed
            self.bot.angular_speed = 1
            self.bot.Stop()
            #Rotate left
            self.bot.RotateLeft()
    #Is the vehicle still running?
    def Running(self):
        #Is the bot running?
        return self.bot.Running()
        
            
        
    

