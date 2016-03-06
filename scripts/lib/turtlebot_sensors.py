#!/usr/bin/env python
import rospy
from time import sleep
from geometry_msgs.msg import Twist
import math


#
# Image proc related imports
#
from cv2 import *
from cv2 import imencode
from cv2 import COLOR_BGR2GRAY
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import base64
import numpy as np


#
# GLOBAL VARIABLES FOR CAMERA TOPICS
#

#KINECT_RGB_TOPIC = "/camera/rgb/image_color"
KINECT_RGB_TOPIC = "turtlebot_1/camera/rgb/image_raw"

#
# Initialise windows for displaying kinect outputs
#
#RGB
namedWindow(KINECT_RGB_TOPIC)
#Start the window thread
startWindowThread()

#Image filter classes
#Blur
class BlurFilter:
    #Initialise
    def __init__(self,mask_size):
        self.mask_size = mask_size
    #Apply filter and return resulting image
    def Apply(self,image):
        if(image!=None):
            #_image = blur(image,(self.mask_size,self.mask_size))
            _image = medianBlur(image,self.mask_size)
            return _image
        else:
            return iamge
#To grayscale
class GrayFilter:
    #Initialise
    def Apply(self,image):
        if(image!=None):
            _image = cvtColor(image,COLOR_BGR2GRAY)
            return _image
        else:
            return image
#Canny edge detector
class CannyFilter:
    #Initialise    
    def __init__(self,low,high):
        self.low = low
        self.high = high
    #Apply
    def Apply(self,image):
        if(image!=None):
            _image = Canny(image,self.low,self.high)
            return _image
        else:
            return image
#Extract filter
class ExtractFilter:
    #Initialise
    def __init__(self,low,high):
        if(low!=None and high!=None):
            #Convert to hsv
            self.low = low
            self.high = high
        else:
            self.low = np.array([0,0,0])
            self.high = np.array([255,255,255])
    #Apply
    def Apply(self,image):
        if(image!=None):
            #Convert to hsv
            hsv = cvtColor(image,COLOR_BGR2HSV)
            #Define the mask
            mask = inRange(hsv,self.low,self.high)
            #Result
            res = bitwise_and(image,image,mask=mask)
            return res
        else:
            return image
#Morph filter
class MorphologicalFilter:
    #Initialise
    def __init__(self,_type,kernel,iterations):
        self._type = _type
        self.kernel = kernel
        self.iterations = iterations
    #Apply
    def Apply(self,image):
        #Only try if image is not null
        if(image!=None and self._type!=None and self.kernel!=None and self.iterations!=None):
            #Holds the result        
            _image = image
            #What type of morphological transformation is this?
            #Dilation
            if(self._type == "dilate" or self._type == "dilation"):
                _image = dilate(image,self.kernel,iterations=self.iterations)
            #Erosion
            elif(self._type == "erode" or self._type == "erosion"):
                _image = erode(image,self.kernel,iterations=self.iterations)
            #Return the result
            return _image
#Shape filter
class ShapeFilter:
    #Initialise
    def __init__(self,shape):
        self.shape = shape


#Threshold filter
class ThreshFilter:
    #Initialise
    def __init__(self,_min,_max):
        self.min = float(_min)
        self.max = float(_max)
    #Apply
    def Apply(self,image):
        hsv_image = cvtColor(image,COLOR_BGR2HSV)
        print(str(hsv_image))
        #Check image is valid
        if(hsv_image!=None):
            rows,cols,depth = hsv_image.shape
            #Apply threshold
            for i in xrange(rows):
                for j in xrange(cols):
                    pix = float(hsv_image[i,j,2])
                    print(pix)
            return image

            
#Filter container class
class FilterStore:
    #Initialise
    def __init__(self):
        self.cont = []
    #Add a filter to the container
    def Add(self,filt):
        self.cont.append(filt)
    #Apply filters to a given image - and return the resulting image
    def Apply(self,image):
        _image = image
        #Cycle through filters - and apply
        for i in range(0,len(self.cont)):
            _filt = self.cont[i]
            _image = _filt.Apply(_image)
        return _image


#Circle shape
class Circle:
    #Initialise
    def __init__(self,x,y,radius,colour,thickness):
        self.x = x
        self.y = y
        self.radius = radius
        self.colour = colour
        self.thickness = thickness
    #Draw
    def Draw(self,image):
        #Only attempt if all parameters are valid
        if(image!=None and x!=None and y!=None and radius!=None and colour!=None and thickness!= None):
            circle(image,(self.x,self.y), self.radius,self.colour, -1)
        else:
            rospy.logerr("Cannot draw circle onto null image")



#Container for displaying shapes onto the image
class ShapeStore:
    #Initialise
    def __init__(self):
        self.cont = []
    #Add a shape to the container
    def Add(self,shape):
        self.cont.append(shape)
    #Draw the shape onto the given image
    def Draw(self,image):
        #Cycle through shapes - and draw
        for i in range(0,len(self.cont)):
            shape = self.cont[i]
            shape.Draw(image)


#A base image topic for dealing with image inputs
class _Image:
    #Initialise
    def __init__(self,window_name,topic):
        #Use exception handling to check for errors while initialising the RGB image
        try:
            #Camera topic
            self.topic = topic
            #Used for converting from Ros image to cv2 image
            self.bridge = CvBridge()
            #Subscriber for gathering the image
            self._sub = rospy.Subscriber(self.topic,Image,self._callback)
            #Holds gathered image
            self.image = None
            #The name of the window for displaying the output
            self.window_name = window_name
            #Filter store
            self.filters = FilterStore()
            #Shape store
            self.shapes = ShapeStore()
            #Wait to ensure the image is gathered
            while self.image==None:
                rospy.loginfo("Waiting for image..")
            rospy.loginfo("Image found!")
            rospy.loginfo("Initialised Image input with topic: " + str(self.topic))
        except Exception as e:
            rospy.logerr(e)
    #Callback function
    def _callback(self,data):
        try:
            #Get the image
            self.image = self.bridge.imgmsg_to_cv2(data,"bgr8")
            #Apply any filters to the image
            if(self.filters!=None):
                self.image = self.filters.Apply(self.image)
            #Draw shapes onto the image
            if(self.shapes!=None):
                self.shapes.Draw(self.image)
            self.Display()
        except Exception as e:
            rospy.logerr(e)
    #__call__ returns the cv2 image
    def __call__(self):
        try:
            return self.image
        except Exception as e:
            rospy.logerr(e)
            return None

    #Returns the width of the image
    def Width(self):
        if(self.image!=None):
            r,c,d = self.image.shape
            return c
        else:
            return 0
    #Returns the height of the image
    def Height(self):
        if(self.image!=None):
            r,c,d = self.image.shape
            return r
        else:
            return 0
    #Returns the depth of the image
    def Depth(self):
        if(self.image!=None):
            w,h,d = self.image.shape
            return d
        else:
            return 0
    #Display the image in an opencv window
    def Display(self):
        if(self.image!=None and self.window_name!=None):
            try:
                imshow(self.window_name,self.image)
            except Exception as e:
                rospy.logerr(e)
                return None
        else:
            rospy.logerr("Image: " + str(self.image))
            rospy.logerr("Window name: " + str(self.window_name))
            rospy.logerr("Cannot display null image")
    #Print image contents to console
    def Print(self):
        if(self.image!=None):
            print(self.image[:,:,0])
    #Object drawing functions
    def drawCircle(self,x,y):
        if(self.image!=None and x!=None and y!=None):
            self.shapes.Add(Circle(x,y,60,(0,0,255)))
        else:
            rospy.logerr("Cannot add circle shape")
    #Convert the image to grayscale
    def Grayscale(self):
        if(self.image!=None):
            rospy.loginfo("Converting image to grayscale..")
            self.filters.Add(GrayFilter())
    #Blur the image using the given mask size
    def Blur(self,m_size):
        if(self.image!=None and m_size!=None):
            rospy.loginfo("Blurring image using mask size: " + str(m_size))
            self.filters.Add(BlurFilter(m_size))
    #Apply a canny edge detector using the given threshold
    def Canny(self,low,high):
        if(self.image!=None and low!=None and high!=None):
            rospy.loginfo("Applying canny edge detector")
            self.filters.Add(CannyFilter(low,high))
    #Extract from image using given range
    def Extract(self,low,high):
        if(self.image!=None and low!=None and high!=None):
            rospy.loginfo("Extracting from image using range")
            rospy.loginfo("Low: " + str(low))
            rospy.loginfo("High: " + str(high))
            #Add extract filter to the filter store
            self.filters.Add(ExtractFilter(low,high))
    def ofColourRange(self,colour):
        low = np.array([0,0,0])
        high = np.array([255,255,255])
        _c = str(colour)
        #red
        if(_c=="red"):
            low = np.array([0/2,50,50])
            high = np.array([5/2,255,255])
        #orange
        if(_c=="orange"):
            low = np.array([10/2,50,50])
            high = np.array([40/2,255,255])
        #yellow
        if(_c=="yellow"):
            low = np.array([45/2,10,50])
            high = np.array([65/2,255,255])
        #green
        if(_c=="green"):
            low = np.array([147/2,10,10])
            high = np.array([160/2,255,255])
        #lightgreen
        if(_c=="lightgreen"):
            low = np.array([120/2,80,10])
            high = np.array([150/2,255,255])
        #blue
        if(_c=="blue"):
            low = np.array([160/2,50,50])
            high = np.array([260/2,255,255])
        return (low,high)
    
    #Apply a filter which allows extraction of a specific colour group
    def extractColour(self,colour):
        if(colour!=None):
            rospy.loginfo("Appyling colour extraction")
            low = np.array([0,0,0])
            high = np.array([255,255,255])
            l,h = self.ofColourRange(str(colour))
            if(l!=None and h!=None):
                low = l
                high = h
            #Call extract function
            self.Extract(low,high)
    #Dilate the image
    def Dilate(self,kernel_size,iterations):
        if(kernel_size!=None and iterations != None):
            rospy.loginfo("Dilating image")
            #Create the kernel
            kernel = np.ones((kernel_size,kernel_size),np.uint8)
            rospy.loginfo("using kernel " + str(kernel))
            #Perform dilation
            self.filters.Add(MorphologicalFilter("dilate",kernel,1))
    #Erode the image
    def Erode(self,kernel_size,iterations):
        if(kernel_size!=None and iterations != None):
            rospy.loginfo("Eroding image")
            #Create the kernel
            kernel = np.ones((kernel_size,kernel_size),np.uint8)
            rospy.loginfo("using kernel " + str(kernel))
            #Perform erosion
            self.filters.Add(MorphologicalFilter("erode",kernel,1))
    #Apply a threshold to the image
    def applyThreshold(self,_min,_max):
        #Check image is valid
        if(self.image!=None):
            rospy.loginfo("Applying threshold to image")
            rospy.loginfo("Min = " + str(_min))
            rospy.loginfo("Max = " + str(_max))
            self.filters.Add(ThreshFilter(int(_min),int(_max)))
    #Create HSV range depending on given colour
    def getHsvThresh(self,colour):
        #FOR REAL 
        for i in range(0,10):
            image = medianBlur(self.image,5)
        #ELSE SIM
        image = self.image
        #Convert image to hsv
        hsv = cvtColor(image,COLOR_BGR2HSV)
        namedWindow("hsv")        
        imshow("hsv",hsv)
        #Check hsv image is valid
        if hsv!=None:
            #Get colour thresh
            colour_range = self.ofColourRange(str(colour))
            #Get hsv thresh
            hsv_thresh = inRange(hsv,colour_range[0],colour_range[1])
            
            #FOR REAL
            for ii in range(0,10):
                hsv_thresh = medianBlur(hsv_thresh,5)
                    
            
            namedWindow("components")
            imshow("components",hsv_thresh)            
            
            #Attempt to return hsv thresh
            return hsv_thresh
        else:
            return None
    #
    # Get connected components in the image depending on given colour
    #
    def getConnectedComponents(self,colour):
        #Get hsv thresh
        hsv_thresh = self.getHsvThresh(colour)
        #Check threshold is valid
        if(hsv_thresh!=None):
            #Find contours in the image
            c,h = findContours(hsv_thresh.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)
            #Attempt to return the contours in the image
            return (c,h)
        else:
            return (None,None)
    #
    # Returns an array containing the connected components in the image
    #
    def countConnectedComponents(self,colour):
        #Find contours in the image
        c,h = self.getConnectedComponents(colour)
        #If valid - return length
        if(c!=None):
            return len(c)
        else:
            rospy.logerr("could not find connected components")
            return 0
    #
    # Returns the bounding rectangle for a connected component
    #
    def componentBoundingRect(self,colour):
        #Find contours in the image
        c,h = self.getConnectedComponents(colour)
        #Valid?
        if(c!=None and (len(c)>0)):
            rospy.loginfo("found connected components: " + str(len(c)))            
            
            #Get the bounding rect
            rect = boundingRect(c[0])
            #Attempt to return the rect
            return rect
        else:
            rospy.logerr("invalid connected components")
            #Return invalid object
            return None
    #
    # Get the center point from an existing bounding rect
    #
    def rectangleCentrePoint(self,rect):
        if(rect!=None):
            x,y,w,h = rect
            if(x!=None and y!=None and w!=None and h!=None):
                x_c = x + (w/2)
                y_c = y + (h/2)
                return (x_c,y_c)
            else:
                return (0,0)
        else:
            return (0,0)
    #
    # Gets the bounding rectangle of a component, and calculates the centre point
    #
    def componentCenterPoint(self,colour):
        x_c,y_c = self.rectangleCentrePoint(self.componentBoundingRect(colour))        
        return (x_c,y_c)
        
        #Get bounding rect
        #x,y,w,h = self.componentBoundingRect(colour)
        #x_c = x + (w/2) #X
        #y_c = y + (h/2) #Y
        #print(x_c)
        #print(y_c)
        #return (x_c,y_c) #Return as point structure
        
    #Draw a rectangle onto the image
    def Rectangle(self,x,y,w,h,colour,thickness):
        rectangle(self.image,(x,y),(w,h),colour,thickness)
        self.Display()
    
    

    # Return whether the image contains objects of a specific colour
    def hasObjects(self,colour,collision_limit,x_limit):
        #Convert image to hsv
        hsv = cvtColor(self.image,COLOR_BGR2HSV)
        #Check validility of hsv image
        if hsv!=None:
            #Get colour thresh        
            colour_range = self.ofColourRange("green")
            #Get hsv range
            hsv_thresh = inRange(hsv,colour_range[0],colour_range[1])
            #Variables to hold results
            turn_left = False #Does the robot need to turn left?
            turn_right = False #Does the robot need to turn right?
            move_forward = False #Does the robot need to move forwards?
            if(hsv_thresh!=None):        
                #Find contours in the image
                c,h = findContours(hsv_thresh.copy(),RETR_TREE,CHAIN_APPROX_SIMPLE)
                #Were contours found?
                if(len(c) == 1):
                    #Contour found - get the bounding rect                    
                    x,y,w,h = boundingRect(c[0])
                    #Get image details
                    rows,cols,depth = self.image.shape
                    if(x_limit==None):
                        x_limit = self.Width()/2
                    #Check object location x
                    #Out of bounds left
                    if x < (x_limit):
                        turn_left = True
                    #Out of bounds right
                    elif x > (cols - x_limit):
                        turn_right = True
                        
                    #check object location y
                    if y < collision_limit:
                        move_forward = False
                    else:
                        move_forward = True
                
        return (turn_left,turn_right,move_forward)
#
# Kinect sensor
#
class Kinect:
    #Initialise
    def __init__(self):
        #Get the RGB image
        self.rgb = _Image(KINECT_RGB_TOPIC,KINECT_RGB_TOPIC)
    #Track object with given colour
    def trackObject(self,colour):
        #Get the bounding rect of the object
        rect = self.rgb.componentBoundingRect(str(colour))
        #Is the bounding rect valid?
        if(rect!=None):
            #Calculate the centre point
            x_c,y_c = self.rgb.rectangleCentrePoint(rect)
            #Standard rect details
            x,y,w,h = rect
            #Check so that objects won't collide
            if(y<100):
                return -1
            
            #Draw rect to show the centre point
            self.rgb.Rectangle(x_c,y_c,x_c+1,y_c+1,(0,0,255),1)
            #radians = math.atan2(x_c,y_c)
            return math.atan2(x_c,y_c)
        else:
            rospy.logerr("component not found")
            return 0
    #Shutdown the kinect - ensures all windows are destroyed
    def Shutdown(self):
        #Destroy all windows
        destroyAllWindows()
        
        
        
    