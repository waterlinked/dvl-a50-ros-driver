#!/usr/bin/env python
import argparse

import rosbag
import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
    

def dvl_callback(json_data):
    """_summary_

    Parameters
    ----------
    data : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    global data
    vx = json_data.velocity.x
    vy = json_data.velocity.y
    vz = json_data.velocity.z
    sec = json_data.header.stamp.secs
    nsec = json_data.header.stamp.nsecs
    
    
    twist = TwistStamped()
    twist.twist.linear.x = vx
    twist.twist.linear.y = vy
    twist.twist.linear.z = vz
    twist.header.frame_id = 'dvl_link'
    twist.header.stamp.secs = sec
    twist.header.stamp.nsecs = nsec
    twist.header.seq = json_data.header.seq

    # Publish the data
    pub = rospy.Publisher('dvl/data_altered', TwistStamped, queue_size=10)
    pub.publish(twist)



def main():
    """_summary_

    Parameters
    ----------
    None.

    Returns
    -------
    None.

    """
    global data
    rospy.init_node('a50_sub', anonymous=False)
    rospy.Subscriber("dvl/data", DVL, dvl_callback)
    # pub = rospy.Publisher('dvl/data_altered', Pose, queue_size=10)
    # pub.publish(data)

    rospy.spin()  # keeps python from exiting until node is stopped


if __name__ == '__main__':
    main()

