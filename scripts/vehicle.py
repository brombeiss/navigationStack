#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
from rostest.msg import Command1
from rostest.msg import Command2
from roboCar import Car

def callback(data):
  rospy.loginfo("Callback vehicle.py "+data.message)
  if data.message == 'FORWARD' :
    car.forward(speed=0.4)
  elif data.message == 'BACKWARD':
    car.backward(speed=0.3)
  elif data.message == 'LEFT':
    car.left(speed=0.55)
  elif data.message == 'RIGHT':
    car.right(speed=0.55)
  elif data.message == 'RIGHT_BY_DEGREES':
    car.right_by_degrees(data.argument)
  elif data.message == 'LEFT_BY_DEGREES':
    car.left_by_degrees(data.argument)
  elif data.message == 'STOP':
    car.stop()
  elif data.message == 'SHUTDOWN':
    print ("Good-Bye")
    car.stop()
    car.shutdown()



def listener():

    rospy.init_node('vehicle', anonymous=True)

    rospy.Subscriber('command', Command1, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    car = Car()
    listener()
