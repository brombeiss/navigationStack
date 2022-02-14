#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    x = msg.pose.pose.position.x  
    y = msg.pose.pose.position.y
    yaw = msg.pose.pose.orientation.z
    w = msg.pose.pose.orientation.w
    x_trans = -y #because of different frame systems from the assumed world and lidar, the actual output from /poseupdate for x and y is switched and a minus sign is attached
    y_trans = -x
    output = format(x_trans,'.2f')+' '+format(y_trans,'.2f')+' '+format(yaw,'.2f')+' '+format(w,'.2f')
    rospy.loginfo(output)

def listener():


    rospy.init_node('localizer', anonymous=True)

    rospy.Subscriber('/poseupdate', PoseWithCovarianceStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()