#!/usr/bin/env python
import argparse

import rosbag
import rospy
import math
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, PoseWithCovarianceStamped
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as R
import json


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
    pub = rospy.Publisher('dvl/converted/twist', TwistStamped, queue_size=10)
    pub.publish(twist)


def dvl_raw_callback(raw_dvl):
    """_summary_

    Args:
        raw_dvl (_type_): _description_
    """
    data = json.loads(raw_dvl.data)
    vx = data["vx"]
    vy = data["vy"]
    vz = data["vz"]
    fom = data["fom"]
    covariance = np.array(data["covariance"]).reshape(1, -1)[0].tolist()
    altitude = data["altitude"]

    pose = PoseWithCovarianceStamped()
    pose.pose.pose.position.x = vx
    pose.pose.pose.position.y = vy
    pose.pose.pose.position.z = vz
    tmp_cov = np.eye(6).reshape(1, -1)[0].tolist()
    tmp_cov[:9] = covariance
    pose.pose.covariance = tmp_cov
    pose.header.frame_id = 'dvl_link'
    pose.header.stamp = rospy.Time.now()

    # Publish the data
    pub = rospy.Publisher('dvl/converted/cov',
                          PoseWithCovarianceStamped, queue_size=10)
    pub.publish(pose)


def main():
    rospy.init_node('dvl2twist', anonymous=False)
    rospy.Subscriber("dvl/data", DVL, dvl_callback)
    rospy.Subscriber("dvl/json_data", String, dvl_raw_callback)
    # pub = rospy.Publisher('dvl/data_altered', Pose, queue_size=10)
    # pub.publish(data)

    rospy.spin()  # keeps python from exiting until node is stopped


if __name__ == '__main__':
    main()
