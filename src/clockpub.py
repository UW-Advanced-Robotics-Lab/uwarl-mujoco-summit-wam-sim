#! /usr/bin/env python
""" `test_camera_rendering.py`

    Copyright UWARL (UW Mechanical and Control Lab).

    @author:  Jack (Jianxiang) Xu
        Contacts    : j337xu@uwaterloo.ca
        Last edits  : July 25, 2022

"""


# python libraries:
import time

import rospy

from rosgraph_msgs.msg import Clock

def pubclock():
    
    first_clock = time.time()
    print(first_clock)
    last_time = time.time()
    clock = Clock()

    pub_clock = rospy.Publisher('/clock',Clock,queue_size=1)

    while time.time() < last_time+0.5:
        pass
    

    while not rospy.is_shutdown():

        time_now = time.time()
        clock.clock = rospy.Time.from_sec(time_now - first_clock)
        pub_clock.publish(clock)

        time.sleep(0.0005)





if __name__ == '__main__':
    rospy.init_node('ClockPublisher')    

    try:
        pubclock()
    except rospy.ROSInterruptException:
        rospy.logerr('ROS Interrupt Exception during initialization')
    except Exception as e:
        rospy.logerr('An error occurred during initialization: {}'.format(e))



