#!/usr/bin/env python
import socket
import json
import rospy
from std_msgs.msg import String
from waterlinked_a50.msg import DVL
from waterlinked_a50.msg import DVLBeam

TCP_IP = "10.42.0.186"
TCP_PORT = 16171
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
oldJson = ""

theDVL = DVL()
theDVLBeam = DVLBeam()
beam0 = theDVLBeam
beam1 = theDVLBeam
beam2 = theDVLBeam
beam3 = theDVLBeam

def getData():
	global oldJson
	contain = 0
	raw_data = ""
	while contain == 0:
		raw_data = raw_data + s.recv(1)
		if 'json_v1"}' in raw_data:
			contain = 1
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('json_v1"}')
	oldJson = raw_data[1]
	raw_data = raw_data[0] + 'json_v1"}'
		
		
	return raw_data
	

def publisher():
	rospy.init_node('a50_pub', anonymous=False)
	pub_raw = rospy.Publisher('dvl/json_data', String, queue_size=10)
	pub = rospy.Publisher('dvl/data', DVL, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
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
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()
		pass
