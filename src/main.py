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
# import time

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
    freq_muj = rospy.get_param("sim_frequency_mujoco")

    update_rate = int(freq_muj)
    rate_plot = 25

    Engine = jx.Mujoco_Engine(
        xml_path        = "/home/tim/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_mobile_wagon_manipulation.xml",
        # xml_path        = "/home/tim/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_mobile_grasping.xml",
        rate_Hz         = update_rate,
        rate_scene      = rate_plot,
        camera_config   = None,
    )

    # Make sure mujoco is real time
    steptime = 1.0/float(update_rate)

    r = rospy.Rate(update_rate)

    i=0
    j=0
    now = rospy.Time.now().to_time()

    simtime = rospy.Time.now().to_time()
    realtime = rospy.Time.now().to_time()
    no_sleep = False


    while not rospy.is_shutdown():
        
        # regulate update freq to be real time:
        Engine._update()
         
        i+=1

        no_sleep = False

        # Check frequency and compare simtime and realtime:
        if  i == 100:

            # Compute sim and real time
            simtime += i*steptime
            i=0
            j+=1

            realtime = rospy.Time.now().to_time()

            # Compute frequency over 100 iterations
            time_taken = realtime-now
            freq = 1/time_taken*100

            if j>20:
                j = 0
                # rospy.loginfo('Mujoco update cannot reach minimum update frequency of: ' + str(1/steptime) + ', actual (Hz): '+ str(freq))
                rospy.loginfo('Simtime: '+ str(simtime)+', Realtime: ' + str(realtime))

            # Do not sleep for 1 step if simulation is lagging behind 
            if simtime < realtime-0.05:
                no_sleep = True

            now = rospy.Time.now().to_time()
        
        if not no_sleep:
            r.sleep()




if __name__ == '__main__':
    rospy.init_node('Mujocolaunch')

    try: 
        main()
    except jx.MuJoCo_Engine_InterruptException:
        pass
