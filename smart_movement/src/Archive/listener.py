#!/usr/bin/python


import rospy 
from std_msgs.msg import String 

def callback(data):
	rospy.loginfo(data)
	rospy.loginfo(data.data)


def listener():
	rospy.init_node('listener' , anonymous=True)
	rospy.Subscriber('encoder_data', String, callback)
	rospy.spin()



if __name__ == '__main__':
	listener()













