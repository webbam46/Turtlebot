#!/usr/bin/env python
from lib.bvehicle import BVehicle
import rospy

#Init
try:
    #Create the braitenberg vehicle
    bot = BVehicle()
    bot.type = 'fear'
    #Start fear behaviour
    while bot.Running():
        bot.Run()
    rospy.spin()
except Exception as e:
    print(e)

