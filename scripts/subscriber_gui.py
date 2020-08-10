#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from Tkinter import *

#GUI Setup
root = Tk()
root.geometry("1150x450")

#Global variables to handle all the parsed JSON data
gTime = DoubleVar(root)
gVX = DoubleVar(root)
gVY = DoubleVar(root)
gVZ = DoubleVar(root)
gFom = DoubleVar(root)
gAltitude = DoubleVar(root)
gVelValid = StringVar(root)
gStatus = StringVar(root)
gForm = StringVar(root)

g0ID = StringVar(root)
g0Vel = DoubleVar(root)
g0Distance = DoubleVar(root)
g0rssi = DoubleVar(root)
g0nsd = DoubleVar(root)
g0valid = StringVar(root)

g1ID = StringVar(root)
g1Vel = DoubleVar(root)
g1Distance = DoubleVar(root)
g1rssi = DoubleVar(root)
g1nsd = DoubleVar(root)
g1valid = StringVar(root)

g2ID = StringVar(root)
g2Vel = DoubleVar(root)
g2Distance = DoubleVar(root)
g2rssi = DoubleVar(root)
g2nsd = DoubleVar(root)
g2valid = StringVar(root)

g3ID = StringVar(root)
g3Vel = DoubleVar(root)
g3Distance = DoubleVar(root)
g3rssi = DoubleVar(root)
g3nsd = DoubleVar(root)
g3valid = StringVar(root)


def callbackRAW(data):
	rospy.loginfo(rospy.get_caller_id() + "Data received: %s", data.data)
	
def callback(data):
	gTime.set('%2.2f' % data.time)
	gVX.set('%2.2f' % data.velocity.x)
	gVY.set('%2.2f' % data.velocity.y)
	gVZ.set('%2.2f' % data.velocity.z)
	gFom.set('%2.2f' % data.fom)
	gAltitude.set('%2.2f' % data.altitude)
	gVelValid.set(str(data.velocity_valid))
	gStatus.set(str(data.status))
	gForm.set(str(data.form))

	g0ID.set(str(data.beams[0].id))
	g0Vel.set(str(data.beams[0].velocity))
	g0Distance.set(str(data.beams[0].distance))
	g0rssi.set(str(data.beams[0].rssi))
	g0nsd.set(str(data.beams[0].nsd))
	g0valid.set(str(data.beams[0].valid))

	g1ID.set(str(data.beams[1].id))
	g1Vel.set(str(data.beams[1].velocity))
	g1Distance.set(str(data.beams[1].distance))
	g1rssi.set(str(data.beams[1].rssi))
	g1nsd.set(str(data.beams[1].nsd))
	g1valid.set(str(data.beams[1].valid))

	g2ID.set(str(data.beams[2].id))
	g2Vel.set(str(data.beams[2].velocity))
	g2Distance.set(str(data.beams[2].distance))
	g2rssi.set(str(data.beams[2].rssi))
	g2nsd.set(str(data.beams[2].nsd))
	g2valid.set(str(data.beams[2].valid))

	g3ID.set(str(data.beams[3].id))
	g3Vel.set(str(data.beams[3].velocity))
	g3Distance.set(str(data.beams[3].distance))
	g3rssi.set(str(data.beams[3].rssi))
	g3nsd.set(str(data.beams[3].nsd))
	g3valid.set(str(data.beams[3].valid))
	
	
	
def subscriber():
	rospy.init_node('a50_sub', anonymous=False)
	rospy.Subscriber("dvl/json_data", String, callbackRAW)
	rospy.Subscriber("dvl/data", DVL, callback)
	
	
	
	
#GUI layout and design
root.title("Waterlinked DVL A50")

label = Label(root, text="Data collected from the DVL")
label.grid(columnspan=5, sticky=W)
label.config(font=("Courier", 26))

label_ph1 = Label(root,  text="            ")
label_ph1.grid(row=1, column=1)
label_ph1.config(font=("Courier", 22))
label_ph2 = Label(root,  text="            ")
label_ph2.grid(row=1, column=2)
label_ph2.config(font=("Courier", 22))
label_ph3 = Label(root,  text="            ")
label_ph3.grid(row=1, column=3)
label_ph3.config(font=("Courier", 22))
label_ph4 = Label(root,  text="            ")
label_ph4.grid(row=1, column=4)
label_ph4.config(font=("Courier", 22))

label_timeT = Label(root,  text="Time:")
label_timeT.grid(row=3, sticky=E)
label_timeT.config(font=("Courier", 22))

label_time = Label(root,  textvariable=gTime)
label_time.grid(row=3, column=1)
label_time.config(font=("Courier", 22))

label_velT = Label(root,  text="Velocity:")
label_velT.grid(row=4, sticky=E)
label_velT.config(font=("Courier", 22))

label_velX = Label(root,  textvariable=gVX)
label_velX.grid(row=4, column=1)
label_velX.config(font=("Courier", 22))
label_velY = Label(root,  textvariable=gVY)
label_velY.grid(row=4, column=2)
label_velY.config(font=("Courier", 22))
label_velZ = Label(root,  textvariable=gVZ)
label_velZ.grid(row=4, column=3)
label_velZ.config(font=("Courier", 22))

label_fomT = Label(root,  text="FOM:")
label_fomT.grid(row=5, sticky=E)
label_fomT.config(font=("Courier", 22))

label_fom = Label(root,  textvariable=gFom)
label_fom.grid(row=5, column=1)
label_fom.config(font=("Courier", 22))

label_altT = Label(root,  text="Altitude:")
label_altT.grid(row=6, sticky=E)
label_altT.config(font=("Courier", 22))

label_alt = Label(root,  textvariable=gAltitude)
label_alt.grid(row=6, column=1)
label_alt.config(font=("Courier", 22))

label_valT = Label(root,  text="Velocity Valid:")
label_valT.grid(row=7, sticky=E)
label_valT.config(font=("Courier", 22))

label_val = Label(root,  textvariable=gVelValid)
label_val.grid(row=7, column=1)
label_val.config(font=("Courier", 22))

label_transT = Label(root, text="Transducers:")
label_transT.grid(row=8, sticky=E)
label_transT.config(font=("Courier", 22))
label_beam0T = Label(root, text="Beam:0", bg="gray80")
label_beam0T.grid(row=8, column=1, sticky='ew')
label_beam0T.config(font=("Courier", 22))
label_beam1T = Label(root, text="Beam:1", bg="gray100")
label_beam1T.grid(row=8, column=2, sticky='ew')
label_beam1T.config(font=("Courier", 22))
label_beam2T = Label(root, text="Beam:2", bg="gray80")
label_beam2T.grid(row=8, column=3, sticky='ew')
label_beam2T.config(font=("Courier", 22))
label_beam3T = Label(root, text="Beam:3", bg="gray100")
label_beam3T.grid(row=8, column=4, sticky='ew')
label_beam3T.config(font=("Courier", 22))

label_beamVelT = Label(root, text="Velocity")
label_beamVelT.grid(row=9, sticky=E)
label_beamVelT.config(font=("Courier", 15))
label_beamVel0 = Label(root, textvariable=g0Vel, bg="gray75")
label_beamVel0.grid(row=9, column=1, sticky='ew')
label_beamVel0.config(font=("Courier", 15))
label_beamVel1 = Label(root, textvariable=g1Vel, bg="gray95")
label_beamVel1.grid(row=9, column=2, sticky='ew')
label_beamVel1.config(font=("Courier", 15))
label_beamVel2 = Label(root, textvariable=g2Vel, bg="gray75")
label_beamVel2.grid(row=9, column=3, sticky='ew')
label_beamVel2.config(font=("Courier", 15))
label_beamVel3 = Label(root, textvariable=g3Vel, bg="gray95")
label_beamVel3.grid(row=9, column=4, sticky='ew')
label_beamVel3.config(font=("Courier", 15))

label_beamDistT = Label(root, text="Distance")
label_beamDistT.grid(row=10, sticky=E)
label_beamDistT.config(font=("Courier", 15))
label_beamDist0 = Label(root, textvariable=g0Distance, bg="gray80")
label_beamDist0.grid(row=10, column=1, sticky='ew')
label_beamDist0.config(font=("Courier", 15))
label_beamDist1 = Label(root, textvariable=g1Distance, bg="gray100")
label_beamDist1.grid(row=10, column=2, sticky='ew')
label_beamDist1.config(font=("Courier", 15))
label_beamDist2 = Label(root, textvariable=g2Distance, bg="gray80")
label_beamDist2.grid(row=10, column=3, sticky='ew')
label_beamDist2.config(font=("Courier", 15))
label_beamDist3 = Label(root, textvariable=g3Distance, bg="gray100")
label_beamDist3.grid(row=10, column=4, sticky='ew')
label_beamDist3.config(font=("Courier", 15))

label_beamRssiT = Label(root, text="rssi")
label_beamRssiT.grid(row=11, sticky=E)
label_beamRssiT.config(font=("Courier", 15))
label_beamRssi0 = Label(root, textvariable=g0rssi, bg="gray75")
label_beamRssi0.grid(row=11, column=1, sticky='ew')
label_beamRssi0.config(font=("Courier", 15))
label_beamRssi1 = Label(root, textvariable=g1rssi, bg="gray95")
label_beamRssi1.grid(row=11, column=2, sticky='ew')
label_beamRssi1.config(font=("Courier", 15))
label_beamRssi2 = Label(root, textvariable=g2rssi, bg="gray75")
label_beamRssi2.grid(row=11, column=3, sticky='ew')
label_beamRssi2.config(font=("Courier", 15))
label_beamRssi3 = Label(root, textvariable=g3rssi, bg="gray95")
label_beamRssi3.grid(row=11, column=4, sticky='ew')
label_beamRssi3.config(font=("Courier", 15))

label_beamNsdT = Label(root, text="nsd")
label_beamNsdT.grid(row=12, sticky=E)
label_beamNsdT.config(font=("Courier", 15))
label_beamNsd0 = Label(root, textvariable=g0nsd, bg="gray80")
label_beamNsd0.grid(row=12, column=1, sticky='ew')
label_beamNsd0.config(font=("Courier", 15))
label_beamNsd1 = Label(root, textvariable=g1nsd, bg="gray100")
label_beamNsd1.grid(row=12, column=2, sticky='ew')
label_beamNsd1.config(font=("Courier", 15))
label_beamNsd2 = Label(root, textvariable=g2nsd, bg="gray80")
label_beamNsd2.grid(row=12, column=3, sticky='ew')
label_beamNsd2.config(font=("Courier", 15))
label_beamNsd3 = Label(root, textvariable=g3nsd, bg="gray100")
label_beamNsd3.grid(row=12, column=4, sticky='ew')
label_beamNsd3.config(font=("Courier", 15))

label_beamValidT = Label(root, text="Beam Valid")
label_beamValidT.grid(row=13, sticky=E)
label_beamValidT.config(font=("Courier", 15))
label_beamValid0 = Label(root, textvariable=g0valid, bg="gray75")
label_beamValid0.grid(row=13, column=1, sticky='ew')
label_beamValid0.config(font=("Courier", 15))
label_beamValid1 = Label(root, textvariable=g1valid, bg="gray95")
label_beamValid1.grid(row=13, column=2, sticky='ew')
label_beamValid1.config(font=("Courier", 15))
label_beamValid2 = Label(root, textvariable=g2valid, bg="gray75")
label_beamValid2.grid(row=13, column=3, sticky='ew')
label_beamValid2.config(font=("Courier", 15))
label_beamValid3 = Label(root, textvariable=g3valid, bg="gray95")
label_beamValid3.grid(row=13, column=4, sticky='ew')
label_beamValid3.config(font=("Courier", 15))


	
if __name__ == '__main__':
	subscriber()
	
	
root.mainloop()
