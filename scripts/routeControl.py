#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rostest.msg import Command1
import numpy as np
import math
import time


def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = math.degrees(np.arctan2(y, x))
    return(rho, phi)

def sensor2cart(x,y):  # translate x,y data from lidar to our own coordinate system
    return(y,-x)

def quaternion2degrees(yaw):
    yaw_in_degrees = - math.degrees(np.arcsin(yaw)*2)
    return yaw_in_degrees



def callback(msg):
    global x0
    global y0
    global yaw0
    global callbackReady
    x = msg.pose.pose.position.x  #because of different frame systems from realworld and lidar, the actual output from /poseupdate is switched
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




def listener():
    
    rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, callback)
    rospy.loginfo("...Running")
    finish = False
    commandoMessage = ""
    rate.sleep()
    starttime = int(time.time())
    while not finish:
        now = int(time.time())
        rospy.loginfo("Time: "+ str(now)+" "+str(starttime))
        if (now - starttime > 15 ):
            finish = True
        
        if (callbackReady):
            # Build Delta 
            dx = x1 - x0
            dy = y1 - y0
            (rho,phi) = cart2pol(dx,dy) #Conversion to Polarcoordinate-System
            distance = rho
            yaw1 = 90.0 - phi 
            dyaw = yaw1-yaw0
            rospy.loginfo("yaw0: "+ str(int(yaw0))+" yaw1: "+ str(int(yaw1))+" dyaw: "+ str(int(dyaw))+ " distance "+format(distance, '.2f'))
            rospy.loginfo("x0: "+ format(x0, '.2f')+" y0: "+format(y0, '.2f'))
            #if commandoMessage == "":
            commandoMessage = ""
            # TURN
            if abs(dyaw) > 10.0: #Tolerance
                if (dyaw >= 0):
                    commandoMessage = "RIGHT"
                    commandoArgument = int(abs(999))
                else:
                    commandoMessage = "LEFT"
                    commandoArgument = int(abs(dyaw)) 
            # FORWARD
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
            if distance < 0.05:
                finish = True

    sendCommand("SHUTDOWN",0)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('routeControl', anonymous=True)
    pub = rospy.Publisher('command', Command1, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    callbackReady = False
    oldMessage = ""
    # Starting Position
    x0 = 0.0
    y0 = 0.0
    yaw0 = 0.0
    # Destination
    x1 = 1.0
    y1 = 1.5

    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Good-Bye")

    