#!/usr/bin/env python3

import rospy
import numpy as np
import pyrealsense2 as rs
from std_msgs.msg import String
from rostest.msg import Command2


#Parameter
const_frame = (640,480)
const_min_depth = 161
const_clipping_frame = [(160,120),(480,360)]  # Half dimension of camera frame




# Create a context object. This object owns the handles to all connected realsense devices
pipeline = rs.pipeline()

# Configure streams
config = rs.config()
config.enable_stream(rs.stream.depth, const_frame[0], const_frame[1], rs.format.z16, 30)

# Start streaming
pipeline.start(config)


def executeCommand(command):
    msg = Command2()
    msg.message = command   
    print(command)   
    pub.publish(msg)
    rate.sleep()

def cameraCheck():
    camera = 'ON'
    if not rospy.is_shutdown():
        while True:
            # This call waits until a new coherent set of frames is available on a device
            # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called

            if camera == 'ON':
                frames = pipeline.wait_for_frames()
                depth = frames.get_depth_frame()
                ## if not depth: continue

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
                                        camera = 'OFF'
                                        executeCommand("TURN")
                                        camera = 'ON'
                                        exit # leave loop
                else:
                    executeCommand("GO")
                    exit 


if __name__ == '__main__':
    pub = rospy.Publisher('command2', Command2, queue_size=10)
    rospy.init_node('obstacelAvoidance', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    try:
        cameraCheck()
    except rospy.ROSInterruptException:
        pass
