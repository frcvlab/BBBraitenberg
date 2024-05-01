#
# BRAITENROS
# A python package to implement braitenberg like
# behavior based programming in the ROS Turtlebot3 T3 physical robot
# also works with the Gazebo simulation
#

# c dml 2020,2021,2022,2023,2024 Fordham University NYC FRCV Lab
#
# Permission is granted to copy this code as long as you include the headers
# and attribution and this message. No support is implied.
#

import math
import random
import sys
import time

random.seed() # initialize the random module

import rospy # ROS module

# ROS message definitions
#
from geometry_msgs.msg import Twist      # ROS Twist message
from nav_msgs.msg import Odometry        # ROS Pose message
from sensor_msgs.msg import LaserScan    # ROS laser msg
from tf.transformations import quaternion_about_axis, euler_from_quaternion


# Useful additional modules
#
import numpy as np


# OpenCV module
#
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Mathplot lib
#
from matplotlib import pyplot as plt

# Threading for multiple robots
#
import threading

#
# Main class is braitenros, member functions for control of one robot
# The class functions as a single ROS node
#
# Braitenberg Callbacks
#     
# def twoMotor2One(self,vl,vr): # change vleft,vright to ROS topic; not called by user directly  
# def doConnections(self,showCamera): # implement connections that have been made; not called by user directly
#
# User callbacks
# def connect(self,source,sink): # User: connect a sensor data source to a and actuator data sink
# def behave(self,showCamera=False,showTouch=False,showLight=False,showPose=False):# User:Carry out braitenberg behavior
# 
# def plotPosition(self): # User: plot positions after the behavior has stopped
# def plotMotors(self): # User: plot the velocities after the behavior has stopped
#     
# Other Non-braitenberg Callbacks
# def callback_Shutdown(self):
# def callback_Pose(self,msg):
# def callback_Image(self,img):
# def callback_Laser(self,msg):
#
# Motion control
# def setVel(self,vl=0.0,va=0.0):
# def goto(self,x,y):
# def spin(self,target_angle):
#
# Multiple robot
# def multipleBehave() -- list of robots
# def multiplePlotPosition() -- list of robots
# ----------------------------------------------


referenceFrame = 'world' # ROS world reference frame
modelName = ''           # no default name
versionname = 'Version 4.2 Decemer 2023'

# model names for multiple robots
modelIndex =0
modelNames=["","T2","T3","T4","T5"] #"" means 1st root compatible with single robot case

# IRM related names for convenience
Released = True

# multiple robot gloal list (for name comparisons)
multipleRobotList=[]

# ALV Class
#
class Braitenros():
        
    # Set up ALV member variables
    # Publishers etc
    #
    def __init__(self, modelName='', simFlag=True, reference_frame=referenceFrame, rate=10,name=""):
        # initiliaze each robot
        global modelIndex,modelNames # names for multiple robot instances
        if modelName=='':
            if modelIndex>=len(modelNames):
                print("Cannot create another robot, sorry")
                return
            modelName = modelNames[modelIndex]
            modelIndex += 1
        self.name=name
        self.modelName=""
        rospy.loginfo(modelName+name+": To stop TurtleBot, type CTRL + C")
        rospy.on_shutdown(self.callback_Shutdown)
        print("Braitenros: ",versionname," Configuring sensors & motors "+modelName+"...")      
        #user controlled diagnostics, flags to enable
        self.showCamera=False
        self.showTouch =False
        self.showLight =False
        self.showPose  =False
        self.showBehavior=True
        self.OneVisualObject=False
         # set up camera image transfer and callback
        print("    CV"),#end=' ');
        self.cvBridge = CvBridge()
        self.cameraImage = Image()
        self.simFlag=simFlag # remember
        if simFlag:
            self.imageTopic= modelName+'/camera/rgb/image_raw'
        else:
            self.imageTopic = modelName+'/raspicam_node/image_raw' # Gazebo: /camera/rgb/image_raw'
        self.image_sub = rospy.Subscriber(self.imageTopic,Image,self.callback_Image)
        
        # set up velocity publishing
        print("OK\n    Motors"),#end=' ')
        self.motionTopic=modelName+'/cmd_vel'
        self.vel_pub = rospy.Publisher(self.motionTopic, Twist, queue_size=0)
        # set up laser callback and emergency 'bumper' sensors
        print("OK\n    Laser"),#end=' ')
        self.laserTopic=modelName+'/scan'
        self.scan_sub = rospy.Subscriber(self.laserTopic, LaserScan, self.callback_Laser)
        # set up odometry callback
        
        print("OK\n    Odometry"),#end=' ')
        self.Pose =[0.0,0.0,0.0]
        self.poseTopic=modelName+'/odom'
        self.pose_sub = rospy.Subscriber(self.poseTopic, Odometry, self.callback_Pose)
        
        print("OK.\nAll done.\nInitializing member variables")
        self._relative_position = [0.0,0.0]
        self._relative_angle = 0.0
        self._relative_roll = 0.0   
        
        #-------------------------------------
        #braitenros variables
        
        # boolean bumper variables
        self.tooClose = 0.5 #meters, how close before the sensor is triggered    
        self.not2Close = 1.5 # meters, something seen but not too close
        self.bumpLeft=False
        self.bumpRight=False
        self.laserReadings=[0]*360
        

        # contact variables 1= contact, 0= no contact
        self.lf_touch, self.rf_touch=0,0
        self.lb_touch, self.rb_touch=0,0
        # detection variables
        self.lf_detect, self.rf_detect=0,0
        self.lb_detect, self.rb_detect=0,0

        
        self.mdq=[0,0,0,0] #lf,lb,rb,rf; min distance quadrants
        self.mdi=[0,0,0,0] #lf,lb,rb,rf; min distance indices
        '''
        self.pmdq=[0,0,0,0] #lf,lb,rb,rf; prev min distance quadrants
        self.pmdi=[0,0,0,0] #lf,lb,rb,rf; pev min distance indices
        self.mdv=[0,0,0,0] #lf,lb,rb,rf; min distance vels
        # detection variables velocity flags
        self.lf_moving, self.rf_moving=0,0
        self.lb_moving, self.rb_moving=0,0
        '''
        # detection variables what name robot was touched
        self.lf_touched, self.rf_touched=0,0
        self.lb_touched, self.rb_touched=0,0
              
        # light variables
        # an intensity value, positive real
        self.lf_light, self.rf_light=0,0
        # target color, this color will be the light stimulus
        self.targetCol = [(200,200,200),(255,255,255)] # default is white paper
        self.target_x = None
        self.target_angle = None
        self.target_centers=[]
        # motor variables
        self.vleft, self.vright=0,0
        
        # connection variables
        self.lf_touch_connect = 1
        self.lb_touch_connect = 2
        self.rb_touch_connect = 3
        self.rf_touch_connect = 4
        
        self.lf_light_connect = 5
        self.rf_light_connect = 6
        
        self.vleft_connect = [] # a list of things that can be connected to the motor
        self.vright_connect = []# unconnected
        
        # Collect data for graphing
        self.poseListX = []
        self.poseListY = []
        self.vrightList = []
        self.vleftList = []
        #--------------------------------
        
        #BBbraitenros
        self.behaviors=[]
        self.behaviorOutput={} # dictionary (vleft,vright)
        self.rateSkip={} # dictionary (behavior,skip,counter)
        self.memory={} # state variables stored here
       
        # set the robot to stationary
        self.rate=rate
        self.modelName=modelName

        print("Braitenros: All done. 3 second delay before behavior starts..")
        rospy.init_node('Braitenrosnode', anonymous=False) # everything is just 1 node
        self.setVel(0.0,0.0)
        rospy.sleep(3)
        return

    # default system shutdown
    def callback_Shutdown(self):
        '''Called with a ^C'''
        print(self.modelName+" Shutting down robot behavior.")
        msg = Twist()
        msg.angular.z=0.0
        msg.linear.x=0.0
        self.vel_pub.publish(msg) 
        return
    
    #Callback for odometry
    def callback_Pose(self,msg):
        
        orient = msg.pose.pose.orientation
        quat = [axis for axis in [orient.x, orient.y, orient.z, orient.w]]
        (roll,pitch,yaw)=euler_from_quaternion(quat)
        self.Pose[0]= msg.pose.pose.position.x
        self.Pose[1]= msg.pose.pose.position.y
        self.Pose[2]= yaw
        return
    
    #convenience function to set the velocity
    def setVel(self,vl=0.0,va=0.0):
        msg = Twist()
        msg.linear.x=np.clip(vl,-0.3,0.3)
        msg.linear.y,msg.linear.z=0.0,0.0
        msg.angular.z=np.clip(va,-0.9,0.9)
        msg.angular.x,msg.angular.y=0.0,0.0
        self.vel_pub.publish(msg)
        return
        
    # change vleft,vright to ROS
    def twoMotor2One(self,vl,vr):
       widthOfRobot = 0.5
       self.setVel( 0.5*(vr+vl), (vr-vl)/widthOfRobot)
       return
       
    # return a float between -0.5 and +.5 step 0.1
    def random(self):
         return float(random.randrange(-5,5))/10.0
    
    #flatten a nestd list
    def flatten(self,list_of_lists):
        if len(list_of_lists) == 0:
            return list_of_lists
        if isinstance(list_of_lists[0], list):
            return self.flatten(list_of_lists[0]) + self.flatten(list_of_lists[1:])
        return list_of_lists[:1] + self.flatten(list_of_lists[1:])
    
    
       
    # add a behavior to the behavior list
    def addBehavior(self,b):
        self.behaviors.append(b)

        blist=self.flatten(self.behaviors)
        print("Behavior List: ["),#,end="")
        #print (blist)
        for b in blist:
            print(b.__name__),#end=" ")
            self.addSkip(b,2)
        print("]")
        return
        
    # add a rate skip for this behavior
    def addSkip(self,b,skip):
        self.rateSkip[b]=[skip,0]
        return
        
    # functions to manipulate the memory dictionary
    
    # assuming name is in memory dictionary, reset its value
    def remember(self,name,value):
       self.memory[name]=value
       #print("Remember ",name," is ",self.memory[name])
       return
    
    # if name is in memory dictionary return its value
    def recall(self,name):
       if name in self.memory:
           #print("Recall ",name," is ",self.memory[name])
           return self.memory[name]
       else:
           print(name," has not been set by any behavior.")
       return 0
       
    # set up a habituated value in memory, with maxup before it
    # triggers and then maxdown before it resets
    # if it exists already, then update for one tick
    
    def usingHabituated(self,name,maxvalup,maxvaldown):
       if not name in self.memory: # does not exist yet
           entry=[name, maxvalup,maxvalup+maxvaldown,0] #last entry is ctr
           self.remember(name,entry)
       elif self.recall(name)[3]>self.recall(name)[2]: #reset ctr
           self.recall(name)[3]=0
       else:
           self.recall(name)[3] += 1 # increment ctr
       return 
    
    # determine if a habituated memory name has passed its 
    # maxup counter, that it, it has triggered. It will stay triggered
    # until it passes its maxdown value
    
    def triggered(self,name):
       trig = self.recall(name)[3]>self.recall(name)[1]
       #if trig:
       #    print(name+"!"),# end=" ")
       return trig
        
    # carry out behavior b 
    def doBB(self,b):
        # is this a behavior or a priority list
        if type(b) is list:
            for sub in b:
                if self.doBB(sub):
                    #print(sub.__name__," is subsuming")
                    break
            return True 
        # is there a skip rate defined for this behavior
        if (b in self.rateSkip):
            if self.rateSkip[b][1]>0:
                self.rateSkip[b][1]-=1 # just counting down
                prior = self.behaviorOutput[b] # vleft,vright,status
                self.vleft=+prior[0]
                self.vright+=prior[1]
                #print("Rate Skipped ",b.__name__)
                flag="-"
                if prior[2]:
                    flag="+"
                if self.showBehavior:
                    print(b.__name__+flag),#end=" ")
                return prior[2]
            else:
                self.rateSkip[b][1]=self.rateSkip[b][0] # recharge it
        # carry out a behavior
        #print(b.__name__)
        vleft,vright = self.vleft, self.vright  # only needed for skip
        status = b(self)
        # remember the effect of this behavior and whether it was released
        self.behaviorOutput[b]=(self.vleft-vleft,self.vright-vright,status)
        #if status:
        #    print("Behavior \""+b.__name__+"\" released.")
        flag="-" #indicate whether the behavior was released or not
        if status:
            flag="+"
        if self.showBehavior:
            print(b.__name__+flag),#end=" ")
        return status
         
    # connect a sensor data source to a and actuator data sink   
    def connect(self,source,sink):
       # add another connection to source for sink
       if not sink in source:
           source.append(sink)
       return
       
    # implement connections
    def doConnections(self,showCamera):
       #showCamera=True
       motor_gain= 0.15
       light_gain = 0.005

       if showCamera:
           cv2.namedWindow("Camera")
       
       #while not rospy.is_shutdown():
       try:
           # propagate sensors to motors
           self.vleft,self.vright=0,0
           cap = motor_gain # first time around, for sense=-1

           for sense in [-1,1]:
               # vleft connection
               # touch sensors
               if sense*self.lf_touch_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.lf_touch*motor_gain
               if sense*self.lb_touch_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.lb_touch*motor_gain
               if sense*self.rb_touch_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.rb_touch*motor_gain
               if sense*self.rf_touch_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.rf_touch*motor_gain
               # light sensors
               if sense*self.rf_light_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.rf_light*light_gain
               if sense*self.lf_light_connect in self.vleft_connect:
                   self.vleft += cap + sense*self.lf_light*light_gain
 
               # vright connection
               # touch sensors
               if sense*self.lf_touch_connect in self.vright_connect:
                   self.vright += cap + sense*self.lf_touch*motor_gain
               if sense*self.lb_touch_connect in self.vright_connect:
                   self.vright += cap + sense*self.lb_touch*motor_gain
               if sense*self.rb_touch_connect in self.vright_connect:
                   self.vright += cap + sense*self.rb_touch*motor_gain
               if sense*self.rf_touch_connect in self.vright_connect:
                   self.vright += cap + sense*self.rf_touch*motor_gain
               # light sensors
               if sense*self.rf_light_connect in self.vright_connect:
                   self.vright += cap + sense*self.rf_light*light_gain
               if sense*self.lf_light_connect in self.vright_connect:
                   self.vright += cap + sense*self.lf_light*light_gain
               # end of connections
               cap = 0 # second time around, for sense=1
           #make it happen
          
           if self.simFlag and (abs(self.Pose[0])>5 or abs(self.Pose[1]>5)):
               self.vleft,self.vright=0.0,0.0
               print("Stopped: Out of bounds [+/-5,+/-5]")
               self.twoMotor2One(self.vleft,self.vright)
               raise(rospy.ROSInterruptException)
          
           #self.twoMotor2One(self.vleft,self.vright)
           
           self.poseListX.append(self.Pose[0])
           self.poseListY.append(self.Pose[1])

           if showCamera: # some diagnostic capability
               cv2.imshow("Camera"+self.modelName, cv2.resize(self.cameraImage,(320,240)))
               ret=cv2.waitKey(2) # HAVE to do this for image to show in the window
               #print("LF:",self.lf_light," RF:",self.rf_light)
               #print("X:",self.Pose[1]," Y:",self.Pose[1])
               #print("Vleft:",self.vleft," Vright:",self.vright)
               
       except:
           pass
           
       return
       
    #  
    #Carry out a braitenberg behavior
    #
    def behave(self,showCamera=False,showTouch=False,showLight=False,showPose=False):
        self.showCamera=showCamera
        self.showTouch =showTouch
        self.showLight =showLight
        self.showPose  =showPose
        print("Braitenros: behavior starting now.")
        try:
            rate = rospy.Rate(self.rate)
            
            while not rospy.is_shutdown():
                leftmotor,rightmotor=0,0
                self.doConnections(showCamera)
                leftmotor,rightmotor=self.vleft,self.vright
                if self.showBehavior:
                    print(self.modelName," ["),#end=" ")
                for b in self.behaviors:
                    self.doBB(b)
                    leftmotor+=self.vleft
                    rightmotor+=self.vright               
                self.twoMotor2One(leftmotor,rightmotor)
                self.vrightList.append(rightmotor)
                self.vleftList.append(leftmotor)
          
                if self.showBehavior:
                    print("]")
                if showPose:
                    print("X,Y,Angle:",self.Pose)
                rate.sleep()
        except rospy.ROSInterruptException:
            print("Braitenros: behavior has been terminated.")
            return
            pass
        print("Braitenros: behavior has been terminated.")
        return
        
    # plot position
    def plotPosition(self):
        if not self.simFlag:
            f=open("position.csv","w")
            for x,y in zip(self.poseListX, self.poseListY):
                f.write(str(x)+","+str(y)+"\n")
            f.close()
            print("Position data written to position.csv")
            return
        plt.scatter(self.poseListX, self.poseListY, marker=".")
        plt.title("Robot position")
        plt.xlabel("X")
        plt.ylabel("Y")
        print("The program will not continue until you kill the plot window")
        plt.show()
        return
        
    #plot the velocities
    def plotMotors(self):
        if not self.simFlag:
            f=open("motors.csv","w")
            for x,y in zip(self.vrightList, self.vleftList):
                f.write(str(x)+","+str(y)+"\n")
            f.close()
            print("Motor data written to motors.csv")
            return
        plt.plot(self.vrightList,label="Right", marker="4")
        plt.plot(self.vleftList, label="Left",marker="3")
        plt.title("Motor speeds")
        plt.xlabel("Time Step")
        plt.ylabel("Speed")
        plt.legend(loc="upper right")
        print("The program will not continue until you kill the plot window")
        plt.show()
        return
        
    # allow the user to select what color will be used to identify a target
    def setColorTarget(self, minColor,maxColor):
        self.targetCol = [ minColor, maxColor ]
   
    #Callback to store the latest image
    def callback_Image(self,img):
        '''Called automatically for each new image'''
        #print("1",end=' ') # estimate sense/action time ratio
        self.cameraImage = self.cvBridge.imgmsg_to_cv2(img, "bgr8")
        src = self.cameraImage
        h, w, c = src.shape
        
        iss=h*w*1
        # make a binary image that is 0 except where the color is in range
        targetImage = cv2.inRange(src,self.targetCol[0],self.targetCol[1])

        #Complex Visual Systems - multiple objects, outputs self.target_centers
        contours, _ = cv2.findContours(targetImage.copy(), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
        self.target_centers = []
        for i in range(len(contours)):
            moments = cv2.moments(contours[i])
            if moments['m00']>0:
                self.target_centers.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                cv2.circle(targetImage, self.target_centers[-1], 3, (0, 0, 255), -1)
                # extract the moments of the target image
                
        #Simplex Visual system -- overall 'light' in image, left/right light sensors and target 'angle'
        m = cv2.moments(targetImage)
        fract = m['m00']/iss
        
        self.target_x=None
        self.target_angle=None # absolute angle
        FOV = 62.5 # FOV in degrees of Pi camera
        self.lf_light,self.rf_light=0,0
        
        if fract>0: # skip if the target image has non nonzero regions
            # how far is the X center of target  from X center of image
            delx = w/2 - m['m10']/m['m00']
            
            self.target_x = float(m['m10']/m['m00']) # x coord of target
            self.target_angle = math.degrees(self.Pose[2]) # robot pose
            self.target_angle -= (FOV/float(w))*self.target_x - FOV/2.0
            
            if delx<-10:
                self.lf_light,self.rf_light = 0,int(fract)
            elif delx>10:
                self.lf_light,self.rf_light = int(fract),0
            else:
                self.lf_light,self.rf_light = int(fract/2),int(fract/2)
                
        if self.showLight: # some diagnostic capability
            cv2.imshow('Target'+self.modelName,cv2.resize(targetImage,(320,240)))
            cv2.waitKey(1)
        return
   
    # set the names associated with each touch
    def setTouchedNames(self):
        global multipleRobotList
        mlen=len(self.laserReadings)
        inc=2*math.pi/mlen # angular inc per laser ray
        names=["","","",""]
        threshold=0.1 # how close to check
        for i in range(0,4): # check each touch to see if it was a robot
            angle = self.mdi[i]*inc
            d = self.mdq[i] if not math.isinf(self.mdq[i]) else 0
            xd= d*math.cos(angle)+self.Pose[0]
            yd= d*math.sin(angle)+self.Pose[1]
            for v in multipleRobotList: # check each root to see if it was this one
                if v!=self and np.hypot(xd-v.Pose[0],yd-v.Pose[1])<threshold:
                    names[i]=v.name
        # set the flags
        self.lf_touched = names[0]
        self.lb_touched = names[1]
        self.rb_touched = names[2]
        self.rf_touched = names[3]
        return
        
    #Callback to process laser data, sets the bumpers
    def callback_Laser(self,msg):
        '''Call back function for laser range data'''
        maxrange=10000
        mlen = len(msg.ranges)
        q = int(mlen/4)
        # filter any ranges of zero as it pollutes min
        self.laserReadings = [ r if (r>0 and not math.isnan(r)) else maxrange for r in msg.ranges ]
        nzranges = self.laserReadings

        #self.pmdq = self.mdq # take a backup
        #self.pmdi = self.mdi
               
        self.mdq[0] = np.nanmin(nzranges[0:q])
        self.mdq[1] = np.nanmin(nzranges[q+1:2*q])
        self.mdq[2] = np.nanmin(nzranges[2*q+1:3*q])
        self.mdq[3] = np.nanmin(nzranges[3*q+1:mlen])
        self.mdi[0] = np.nanargmin(nzranges[0:q])
        self.mdi[1] = np.nanargmin(nzranges[q+1:2*q])
        self.mdi[2] = np.nanargmin(nzranges[2*q+1:3*q])
        self.mdi[3] = np.nanargmin(nzranges[3*q+1:mlen])
        '''
        # calculate contact velocities
        for i in range(0,4):
            if self.mdi[i]==self.pmdi[i]: # same contact
                self.mdv[i] = abs(self.mdq[i]-self.pmdq[i])  
            else:
                self.mdv[i]=0
        moving_threshold =0.1
        self.lf_moving = self.mdv[0]>moving_threshold
        self.lb_moving = self.mdv[1]>moving_threshold
        self.rb_moving = self.mdv[2]>moving_threshold
        self.rf_moving = self.mdv[3]>moving_threshold
        '''
        self.setTouchedNames() # set the xx_touched member variables
        
        #include for backwards compatability
        self.bumpLeft  = bool(self.mdq[0]<self.tooClose)
        self.bumpRight = bool(self.mdq[3]<self.tooClose)
        
        # set the contact flags
        self.lf_touch,self.rf_touch,self.lb_touch,self.rb_touch=0,0,0,0
        self.lf_detect,self.rf_detect,self.lb_detect,self.rb_detect=0,0,0,0
        if self.mdq[0]<self.tooClose:
            self.lf_touch=self.mdq[0] 
        elif self.mdq[0]<self.not2Close:
            self.lf_detect=self.mdq[0]    
        if self.mdq[1]<self.tooClose:
            self.lb_touch=self.mdq[1]
        elif self.mdq[1]<self.not2Close:
            self.lb_detect=self.mdq[1]
        if self.mdq[2]<self.tooClose:
            self.rb_touch=self.mdq[2]
        elif self.mdq[2]<self.not2Close:
            self.rb_detect=self.mdq[2]
        if self.mdq[3]<self.tooClose:
            self.rf_touch=self.mdq[3]
        elif self.mdq[3]<self.not2Close:
            self.rf_detect=self.mdq[3]
        if self.showTouch:
            print(" Touch LF,RF,LB,RB: {:.2f},{:.2f},{:.2f},{:.2f}".format(self.lf_touch,self.rf_touch, self.lb_touch, self.rb_touch))
            print(" Detect LF,RF,LB,RB: {:.2f},{:.2f},{:.2f},{:.2f}".format(self.lf_detect,self.rf_detect, self.lb_detect, self.rb_detect))
        return

    #Move the robot to position x,y using odometry
    def goto(self,x,y):
        #
        threshold = 1.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and np.hypot(x-self.Pose[0],y-self.Pose[1])>threshold:
            delTheta = math.atan2(y-self.Pose[1], x-self.Pose[0]) - self.Pose[2]
            # check for angle 'wrapping around'
            if delTheta<-math.pi and self.Pose[2]>math.pi:
               delTheta = (2*math.pi+delTheta)
            #print delTheta, self.Pose[2]
            delDist = np.hypot(x-self.Pose[0],y-self.Pose[1])
            self.setVel( 0.1*delDist, 1.5*delTheta)
            rate.sleep()
        print  ("Move Done ","d=",round(np.hypot(x-self.Pose[0],y-self.Pose[1]),2))
        print  ("Loc= ",round(self.Pose[0],2), round(self.Pose[1],2)) 
        self.setVel(0.0,0.0)
        return   
        
        
    #Spin the robot to an absolute pose (in world angles)
    def spin(self,target_angle):
        accuracy = 0.01 # radians
        adist = abs(self.Pose[2]-target_angle)
        vgain=1.5
        direction2go = 1
        if target_angle<self.Pose[2]: # turn +ve/CCW or -ve/CW
            direction2go = -1 
        
        print ("Spinning to ",target_angle, " from ",self.Pose[2]," vgain= ",vgain)
        rate=rospy.Rate(20)
        while not rospy.is_shutdown() and adist>accuracy:
            adist = abs(self.Pose[2]-target_angle)
            self.setVel(0.0,direction2go*vgain*adist)
            rate.sleep()

        print ("Spin complete at ",self.Pose[2])
        self.setVel(0.0,0.0)
        return

    
    # return the smallest angle to turn - degrees
    def efficientAngle(self, turn_angle):
        if turn_angle > 360 or turn_angle < -360:
            sign = turn_angle/abs(turn_angle)
            mod_turn = turn_angle%360
            turn_angle = sign * mod_turn
        if turn_angle > 180:
            shift_turn = turn_angle - 360
        elif turn_angle < -180:
            shift_turn = turn_angle + 360
        else:
            shift_turn = turn_angle
        return shift_turn 


  
    # function used for positioning robots
    # stationary unless turnFlag is true, then rotating
    def look(self,turnFlag):    
        print(self.modelName," Looking..")
        cv2.namedWindow('Camera '+self.modelName, cv2.WINDOW_AUTOSIZE)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if turnFlag:
                self.setVel(0.0,0.1)
            else:
                self.setVel(0.0,0.0)
            cv2.imshow('Camera '+self.modelName, cv2.resize(self.cameraImage,(320,240)))
            ret=cv2.waitKey(1) # HAVE to do this for image to show in the window
            if ret==27: #ESC
                break
            rate.sleep()
        return



#multiple behave does a threaded behave for every vehicle
#ends when they all end
#

def multipleBehave(vlist):
    global multipleRobotList
    multipleRobotList=vlist
    threadList=[]
    for v in vlist:
        t = threading.Thread(target=v.behave, args=(v.showCamera,v.showTouch,v.showLight,v.showPose))
        threadList.append(t)
        v.showBehavior=False
    for t in threadList:
        t.start()
    for t in threadList:
        t.join()
    #for v in vlist:
    #    v.callback_Shutdown()
    return
   
# multiplePlot will plot all the robot positions on  one graph
#
def multiplePlotPosition(vlist):
    for v in vlist:
        plt.scatter(v.poseListX, v.poseListY, marker=".")
    plt.title("Robot position")
    plt.xlabel("X")
    plt.ylabel("Y")
    print("The program will not continue until you kill the plot window")
    plt.show()
    return
  
   
#----------------------------------END-----------------------------





