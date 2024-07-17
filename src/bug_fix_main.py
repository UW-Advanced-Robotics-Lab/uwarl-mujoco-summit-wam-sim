#! /usr/bin/env python
""" `test_camera_rendering.py`

    Copyright UWARL (UW Mechanical and Control Lab).

    @author:  Jack (Jianxiang) Xu
        Contacts    : j337xu@uwaterloo.ca
        Last edits  : Nov 28, 2023 (Tim van Meijel)

"""
#===================================#
#  I M P O R T - L I B R A R I E S  #
#===================================#

# python libraries:
# import time
import os

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


#===========#
#  M A I N  #
#===========#
def main():
    home_path = os.environ["HOME"]

    # Get update frequency of engine from launch file "mujocolaunch.launch"
    freq_muj = rospy.get_param("sim_frequency_mujoco")
    update_rate = int(freq_muj)
    steptime = 1.0/float(update_rate)
    r = rospy.Rate(update_rate)

    # Define viewer refresh rate
    rate_plot = 10

    rospy.sleep(1.0)

    Engine = jx.Mujoco_Engine(
        xml_path        = home_path+"/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_bug_fix.xml",
        rate_Hz         = update_rate,
        rate_scene      = rate_plot,
        camera_config   = None,
    )

    # Initialize variables
    i=0
    j=0
    # now = rospy.Time.now().to_time()
    simtime = rospy.Time.now().to_time()
    realtime = rospy.Time.now().to_time()
    no_sleep = False

    # Engine update loop which tries to be real time. Checks the time difference between clocktime and simtime
    while not rospy.is_shutdown():
        
        # Step engine to progress in time
        Engine._update()
        
        # Set simulation time counter +1
        i+=1

        # Set no_sleep variable to false to make sure the loop is kept at constant frequency
        no_sleep = False

        # Check frequency and compare simtime and realtime (every 100 iterations):
        if  i == 100:

            # Compute simulation and clock time
            simtime += i*steptime
            realtime = rospy.Time.now().to_time()

            # Set counter to 0 and increase counter for printing simulation and real time
            i=0
            j+=1

            ## Compute frequency over 100 iterations (to be printed below)
            # time_taken = realtime-now
            # freq = 1/time_taken*100

            # Print simulation time and clock time approximately every 10 seconds
            if j>20:
                j = 0
                # rospy.loginfo('Mujoco update cannot reach minimum update frequency of: ' + str(1/steptime) + ', actual (Hz): '+ str(freq))
                rospy.logwarn('Simtime: '+ str(round(simtime,3))+', Realtime: ' + str(round(realtime,3)))

            # Do not sleep for 1 step if simulation is lagging behind. This is done to avoid laging behind. 
            # When simulation lags more than 0.01 seconds behind, it will slowly catch up by not sleeping for 1 iteration 
            # every 100 iterations. This will slowly merge the simulation time with the real time. 
            if simtime < realtime-0.01:
                no_sleep = True

            ## Used for computing frequency
            # now = rospy.Time.now().to_time()
        
        # Check boolian to sleep or not which acts as time regulator
        if not no_sleep:
            r.sleep()




if __name__ == '__main__':
    rospy.init_node('Mujocolaunch')

    try: 
        main()
    except jx.MuJoCo_Engine_InterruptException:
        pass
