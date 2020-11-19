#!/usr/bin/env python
import socket
import json
import rospy
from time import sleep
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
import select

def connect():
	global s, TCP_IP, TCP_PORT
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((TCP_IP, TCP_PORT))
		s.settimeout(1)
	except socket.error as err:
		rospy.logerr("No route to host, DVL might be booting? {}".format(err))
		sleep(1)
		connect()

oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()

def getData():
	global oldJson, s
	raw_data = ""

	while not '\n' in raw_data:
		try:
			rec = s.recv(1) # Add timeout for that
			if len(rec) == 0:
				rospy.logerr("Socket closed by the DVL, reopening")
				connect()
				continue
		except socket.timeout as err:
			rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
			connect()
			continue
		raw_data = raw_data + rec
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('\n')
	oldJson = raw_data[1]
	raw_data = raw_data[0]
	return raw_data
	

def publisher():
	pub_raw = rospy.Publisher('dvl/json_data', String, queue_size=10)
	pub = rospy.Publisher('dvl/data', DVL, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
		if do_log_raw_data:
			rospy.loginfo(raw_data)
		data = json.loads(raw_data)
		pub_raw.publish(raw_data)
		
		theDVL.header.stamp = rospy.Time.now()
		theDVL.header.frame_id = "dvl_link"
		theDVL.time = data["time"]
		theDVL.velocity.x = data["vx"]
		theDVL.velocity.y = data["vy"]
		theDVL.velocity.z = data["vz"]
		theDVL.fom = data["fom"]
		theDVL.altitude = data["altitude"]
		theDVL.velocity_valid = data["velocity_valid"]
		theDVL.status = data["status"]
		theDVL.form = data["format"]
		
		beam0.id = data["transducers"][0]["id"]
		beam0.velocity = data["transducers"][0]["velocity"]
		beam0.distance = data["transducers"][0]["distance"]
		beam0.rssi = data["transducers"][0]["rssi"]
		beam0.nsd = data["transducers"][0]["nsd"]
		beam0.valid = data["transducers"][0]["beam_valid"]
		
		beam1.id = data["transducers"][1]["id"]
		beam1.velocity = data["transducers"][1]["velocity"]
		beam1.distance = data["transducers"][1]["distance"]
		beam1.rssi = data["transducers"][1]["rssi"]
		beam1.nsd = data["transducers"][1]["nsd"]
		beam1.valid = data["transducers"][1]["beam_valid"]
		
		beam2.id = data["transducers"][2]["id"]
		beam2.velocity = data["transducers"][2]["velocity"]
		beam2.distance = data["transducers"][2]["distance"]
		beam2.rssi = data["transducers"][2]["rssi"]
		beam2.nsd = data["transducers"][2]["nsd"]
		beam2.valid = data["transducers"][2]["beam_valid"]
		
		beam3.id = data["transducers"][3]["id"]
		beam3.velocity = data["transducers"][3]["velocity"]
		beam3.distance = data["transducers"][3]["distance"]
		beam3.rssi = data["transducers"][3]["rssi"]
		beam3.nsd = data["transducers"][3]["nsd"]
		beam3.valid = data["transducers"][3]["beam_valid"]
		
		theDVL.beams = [beam0, beam1, beam2, beam3]
		
		pub.publish(theDVL)
		
		rate.sleep()

if __name__ == '__main__':
	global s, TCP_IP, TCP_PORT, do_log_raw_data
	rospy.init_node('a50_pub', anonymous=False)
	TCP_IP = rospy.get_param("~ip", "10.42.0.186")
	TCP_PORT = rospy.get_param("~port", 16171)
	do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
	connect()
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()
