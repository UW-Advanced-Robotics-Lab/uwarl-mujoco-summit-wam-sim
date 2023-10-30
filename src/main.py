#! /usr/bin/env python
""" `test_camera_rendering.py`

    Copyright UWARL (UW Mechanical and Control Lab).

    @author:  Jack (Jianxiang) Xu
        Contacts    : j337xu@uwaterloo.ca
        Last edits  : July 25, 2022

"""
#===================================#
#  I M P O R T - L I B R A R I E S  #
#===================================#

# python libraries:
import time

# python 3rd party libraries:
import numpy as np

import rospy

# Ours:
import mujoco_engine.core_engine as jx


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# #        ___           ___         ___          ___           ___           ___       # #
# #       /__/\         /__/\       /  /\        /  /\         /  /\         /  /\      # #
# #      |  |::\        \  \:\     /  /:/       /  /::\       /  /:/        /  /::\     # #
# #      |  |:|:\        \  \:\   /__/::\      /  /:/\:\     /  /:/        /  /:/\:\    # #
# #    __|__|:|\:\   ___  \  \:\  \__\/\:\    /  /:/  \:\   /  /:/  ___   /  /:/  \:\   # #
# #   /__/::::| \:\ /__/\  \__\:\    \  \:\  /__/:/ \__\:\ /__/:/  /  /\ /__/:/ \__\:\  # #
# #   \  \:\~~\__\/ \  \:\ /  /:/     \__\:\ \  \:\ /  /:/ \  \:\ /  /:/ \  \:\ /  /:/  # #
# #    \  \:\        \  \:\  /:/      /  /:/  \  \:\  /:/   \  \:\  /:/   \  \:\  /:/   # #
# #     \  \:\        \  \:\/:/      /__/:/    \  \:\/:/     \  \:\/:/     \  \:\/:/    # #
# #      \  \:\        \  \::/       \__\/      \  \::/       \  \::/       \  \::/     # #
# #       \__\/         \__\/                    \__\/         \__\/         \__\/      # #
# #                                                                                     # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #


#=======================#
#  D E F I N I T I O N  #
#=======================#


#===================#
#  C O N S T A N T  #
#===================#



#===============================#
#  I N I T I A L I Z A T I O N  #
#===============================#


#==================================#
#  P U B L I C    F U N C T I O N  #
#==================================#


#====================================#
#  P R I V A T E    F U N C T I O N  #
#====================================#

#===========#
#  M A I N  #
#===========#
def main():
    Engine = jx.Mujoco_Engine(
        xml_path        = "/home/tim/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_mobile_wagon_manipulation.xml",
        # xml_path        = "/home/tim/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_mobile_grasping.xml",
        rate_Hz         = 50,
        camera_config   = None,
    )

    rospy.sleep(1)

    # Make sure mujoco is real time
    steptime = 0.002*10
    r = rospy.Rate(1/steptime)
    i=0
    now = rospy.Time.now()
    simtime = 0.0
    realtime = 0.0
    realstart = float(rospy.Time.now().secs)+float(rospy.Time.now().nsecs)/10**9

    while not rospy.is_shutdown():
        
        Engine._update()
        r.sleep() 
        i+=1

        # Check frequency and compare simtime and realtime:
        if  i == 100:

            # Compute sim and real time
            simtime += steptime*i
            i=0
            realtime = float(rospy.Time.now().secs)+float(rospy.Time.now().nsecs)/10**9 - realstart
            # Compute frequency over 
            second = rospy.Time.now()
            time_taken = second-now
            freq = 1/(float(time_taken.secs)+float(time_taken.nsecs)/10**9+0.000000001)*100.0
            # Print if frequency deviates from desired
            if freq < 1/steptime-1.0:
                rospy.logwarn('Mujoco update cannot reach minimum update frequency of: ' + str(1/steptime) + ', actual (Hz): '+ str(freq))
                rospy.loginfo('Simtime: '+ str(simtime)+', Realtime: ' + str(realtime))
            now = rospy.Time.now()




if __name__ == '__main__':
    rospy.init_node('Mujocolaunch')

    try: 
        main()
    except jx.MuJoCo_Engine_InterruptException:
        pass
