#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rostest.msg import Command1
import numpy as np
import math
import time
import pyrealsense2 as rs


def get_time():
    return int(time.time())


# Function for getting the distance via theorem of pythagoras plus calculating the angel between x-axis and hypotnenuse
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = math.degrees(np.arctan2(y, x))
    return(rho, phi)


# Translate x,y data from lidar to my own coordinate system
def sensor2cart(x,y):
    return(y,-x)


# Because the angels are given as quaternions by the topic /poseupdate from hector_slam, they have to be converted to angels in degree
def quaternion2degrees(yaw):
    yaw_in_degrees = - math.degrees(np.arcsin(yaw)*2)
    return yaw_in_degrees


# Getting information about current position and orientation 
def callback(msg):
    global x0
    global y0
    global yaw0
    global callbackReady
    x = msg.pose.pose.position.x 
    y = msg.pose.pose.position.y
    yaw = msg.pose.pose.orientation.z
    (x0,y0) = sensor2cart(x,y)
    yaw0 = quaternion2degrees(yaw)
    callbackReady = True


def sendCommand(message,argument):
    global oldMessage
    if (message != oldMessage):
        call = Command1()
        call.message = message
        call.argument = argument
        pub.publish(call)
        rospy.loginfo(call.message+" "+format(call.argument,'.2f'))
        oldMessage = message



def check4Collision():
    global const_frame,const_clipping_frame,const_min_depth
    global pipeline

    frames = pipeline.wait_for_frames()
    depth = frames.get_depth_frame()
   
    depth_image = np.asanyarray(depth.get_data())
    depth_without_zero = np.ma.masked_array(depth_image,mask=depth_image==0)

    min_dist = np.amin(depth_without_zero)
    if min_dist <= const_min_depth:
        index = np.where(depth_without_zero == min_dist)
        xvalues = index[0]
        yvalues = index[1]
        # Check if the min. dist is in the cutted camera frame
        for i in range(0, len(xvalues),1):
            x = xvalues[i]
            y = yvalues[i]
            if x >= const_clipping_frame[0][0]:
                if x <= const_clipping_frame[1][0]:
                    if y >= const_clipping_frame[0][1]:
                        if y <= const_clipping_frame[1][1]:
                            return True
    return False


def listener():
    
    rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, callback)
    finish = False
    commandoMessage = ""
    rate.sleep()
    suspend_turn_until = get_time()
    starttime = get_time()
    while not finish:
        now = get_time()
        rospy.loginfo("Time: "+ str(now)+" "+str(starttime))
        if (now - starttime > 25 ): # Shutdown after a given time
            finish = True
        
        if (callbackReady):
            # Build Delta of current point and point of destination
            dx = x1 - x0
            dy = y1 - y0
            (rho,phi) = cart2pol(dx,dy) # Conversion to Polarcoordinate-System
            distance = rho
            yaw1 = 90.0 - phi # Actual angle by which the car has to be rotated
            dyaw = yaw1-yaw0 # Current angle by which the car has to be rotated
            # Output of information about position and distance/angel which has to be made
            rospy.loginfo("yaw0: "+ str(int(yaw0))+" yaw1: "+ str(int(yaw1))+" dyaw: "+ str(int(dyaw))+ " distance "+format(distance, '.2f'))
            rospy.loginfo("x0: "+ format(x0, '.2f')+" y0: "+format(y0, '.2f'))
            commandoMessage = ""
            # TURN
            if ( now > suspend_turn_until ):
                if abs(dyaw) > 10.0: #Tolerance
                    if (dyaw >= 0):
                        commandoMessage = "RIGHT"
                        commandoArgument = int(abs(999))
                    else:
                        commandoMessage = "LEFT"
                        commandoArgument = int(abs(999)) 
            # FORWARD
            collision = check4Collision()
            if collision == True:
                sendCommand("RIGHT_BY_DEGREES",90)
                time.sleep(2)
                suspend_turn_until = now + 3   # No turn for the next 5 seconds
            else:
                if commandoMessage != "RIGHT" and commandoMessage != "LEFT":
                    if abs(distance) > 0.1:
                        commandoMessage = "FORWARD"
                        commandoArgument = 0
                        sendCommand(commandoMessage,commandoArgument)
            # NO COMMAND SO FAR -> STOP
            if commandoMessage != "":
                sendCommand(commandoMessage,commandoArgument)
            else:
                commandoMessage = "STOP"
                commandoArgument = 0
                sendCommand(commandoMessage,commandoArgument)
            # TARGET REACHED
            if distance < 0.05: # Tolerance
                finish = True

    sendCommand("SHUTDOWN",0)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('routeControl', anonymous=True)
    pub = rospy.Publisher('command', Command1, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # DEPTH CONTROL
    ##Parameter
    const_frame = (640,480)
    const_min_depth = 170
    const_clipping_frame = [(160,120),(480,360)]  # Half dimension of camera frame
    ## Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    ## Configure streams
    configCamera = rs.config()
    configCamera.enable_stream(rs.stream.depth, const_frame[0], const_frame[1], rs.format.z16, 30)

    ## Start streaming
    pipeline.start(configCamera)


    # MAIN
    callbackReady = False
    oldMessage = ""
    # Starting Position
    x0 = 0.0
    y0 = 0.0
    yaw0 = 0.0
    # Destination
    x1 = 0.0
    y1 = 2.0

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Good-Bye")

    