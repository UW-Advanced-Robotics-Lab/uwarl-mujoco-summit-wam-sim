#!/usr/bin/env python 
import numpy as np
from matplotlib import pyplot as plt
import rospy
# from qualisys.msg import Subject
from gazebo_msgs.msg import LinkStates

from geometry_msgs.msg import Twist

def plot_x(msg):
    global counter
    index = msg.name.index('smt/base_link')
    velx = msg.twist[index].linear.x
    if counter % 10 == 0:
        # stamp = msg.header.stamp
        # time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(counter/10,velx,"*")
        # plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter += 1

def plot_ref(msg):
    global counter1
    # index = msg.name.index('smt/base_link')
    velx = msg.linear.x
    if counter1 % 1 == 0:
        # stamp = msg.header.stamp
        # time = stamp.secs + stamp.nsecs * 1e-9
        plt.plot(counter,velx,'*')
        # plt.axis("equal")
        plt.draw()
        plt.pause(0.00000000001)

    counter1 += 1

if __name__ == '__main__':
    counter = 0
    counter1 = 0

    rospy.init_node("plotter")
    rospy.Subscriber("/mujoco/link_states", LinkStates, plot_x)
    # rospy.Subscriber("/uwarl_a/robotnik_base_control/cmd_vel", Twist, plot_ref)
    # fig, axs = plt.subplots(2)
    plt.ion()
    plt.show()
    rospy.spin()