#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from rostest.msg import Command1
from pynput import keyboard


def executeCommand(command):
    msg = Command1()
    msg.message = command   
    print(command)   
    pub.publish(msg)
    rate.sleep()


def on_press(key):
    if not rospy.is_shutdown():
        if key.char == "w":
            executeCommand("FORWARD")
        if key.char == "y":
            executeCommand("BACKWARD")
        if key.char == "a":
            executeCommand("LEFT")
        if key.char == "d":
            executeCommand("RIGHT")
        if key.char == "s":
            executeCommand("STOP")
        if key.char == "q":
            executeCommand("SHUTDOWN")
            return False

def talker():
   
    with keyboard.Listener( on_press = on_press) as listener:
        listener.join()
        

if __name__ == '__main__':
    command = "STOP"
    rospy.init_node('remoteControl', anonymous=True)
    pub = rospy.Publisher('command', Command1, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
