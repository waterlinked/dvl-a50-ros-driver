#!/usr/bin/env python
import argparse

import rosbag
import rospy
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL, DVLBeam
import matplotlib.pyplot as plt

def main():
    rospy.init_node('dvl_log_read', anonymous=False)
    bag_file = rospy.get_param('~bag', '')

    fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharey = True)
    # gs = axs[0,1].get_gridspec()
    # axs[0,1].remove()
    # axs[0,0].remove()
    # axbig = fig.add_subplot(gs[0,:])
    # fig.tight_layout()

    vxs, vys, vzs = [], [], []
    beam1_d, beam2_d, beam3_d, beam4_d = [], [], [], []
    beam1_v, beam2_v, beam3_v, beam4_v = [], [], [], []

    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics=['/dvl/json_data', '/dvl/data']):
            if topic == '/dvl/data':
                
                # Log the velocities in the body frame of the robot
                vx = -msg.velocity.y
                vy = -msg.velocity.x
                vz = msg.velocity.z
                vxs.append(vx)
                vys.append(vy)
                vzs.append(vz)

                # Log the beams
                beams = msg.beams
                beam1_v.append(beams[0].velocity)
                beam2_v.append(beams[1].velocity)
                beam3_v.append(beams[2].velocity)
                beam4_v.append(beams[3].velocity)

                beam1_d.append(beams[0].distance)
                beam2_d.append(beams[1].distance)
                beam3_d.append(beams[2].distance)
                beam4_d.append(beams[3].distance)

    axs[0].set_title('Velocities (Body Frame)')
    # Change the title font size
    axs[0].title.set_fontsize(10)
    axs[0].set_ylabel('Velocity (m/s)')
    axs[0].plot(vxs, label=r'$v_x$')
    axs[0].plot(vys, label=r'$v_y$')
    axs[0].plot(vzs, label=r'$v_z$')
    axs[0].legend()

    axs[1].set_title('Velocities (Beams)')
    axs[1].set_xlabel('Time (a.u.)')
    axs[1].set_ylabel('Velocity (m/s)')
    axs[1].title.set_fontsize(10)

    axs[1].plot(beam1_v, label='Beam 1 Velocity')
    axs[1].plot(beam2_v, label='Beam 2 Velocity')
    axs[1].plot(beam3_v, label='Beam 3 Velocity')
    axs[1].plot(beam4_v, label='Beam 4 Velocity')
    axs[1].legend()

    plt.show()


if __name__ == '__main__':
    main()