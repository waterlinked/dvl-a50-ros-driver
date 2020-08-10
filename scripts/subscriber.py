#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam

def callbackRAW(data):
	rospy.loginfo(rospy.get_caller_id() + "Data received: %s", data.data)
	
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "Time received: %s",data.time)
	
def subscriber():
	rospy.init_node('a50_sub', anonymous=False)
	rospy.Subscriber("dvl/json_data", String, callbackRAW)
	rospy.Subscriber("dvl/data", DVL, callback)
	
	rospy.spin() #keeps python from exiting until node is stopped
	
if __name__ == '__main__':
	subscriber()
