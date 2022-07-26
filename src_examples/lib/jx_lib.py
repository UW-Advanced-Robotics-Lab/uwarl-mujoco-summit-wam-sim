#! /usr/bin/env python
""" `jx_lib.py`

    Copyright UWARL (UW Mechanical and Control Lab).

    @author:  Jack (Jianxiang) Xu
        Contacts    : j337xu@uwaterloo.ca
        Last edits  : July 25, 2022

"""
#===================================#
#  I M P O R T - L I B R A R I E S  #
#===================================#

# python libraries:
from enum import Enum
import os

import threading, time, signal
from datetime import timedelta


# python 3rd party libraries:
import numpy as np
import matplotlib.pyplot as plt

import mujoco
import mujoco_viewer

import cv2

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


#==================================#
#  P U B L I C    F U N C T I O N  #
#==================================#

def get_files(DIR:str, file_end:str=".png"):
    return [ os.path.join(DIR, f) for f in os.listdir(DIR) if f.endswith(file_end) ]

def create_all_folders(DIR:str):
    path_ = ""
    for folder_name_ in DIR.split("/"):
        path_ = os.path.join(path_, folder_name_)
        create_folder(path_, False)

def clean_folder(DIR:str):
    create_folder(DIR=DIR, clean=True)

def create_folder(DIR:str, clean:bool=False):
    if not os.path.exists(DIR):
        os.mkdir(DIR)
    elif clean:
        filelist = get_files(DIR)
        for f in filelist:
            os.remove(f)


def output_plot(
    data_dict,
    Ylabel  = "",
    Xlabel  = "",
    figsize = (12,6),
    OUT_DIR = "",
    tag     = ""
):
    fig = plt.figure(figsize=figsize)
    for name_, data_ in data_dict.items():
        plt.plot(data_["x"], data_["y"], label=name_) 
    plt.ylabel(Ylabel)
    plt.xlabel(Xlabel)
    plt.legend()
    plt.title("Plot [{}]".format(tag))
    fig.savefig("{}/plot_{}.png".format(OUT_DIR, tag), bbox_inches = 'tight')
    plt.close(fig)
    return fig


def imgs_plot(
        dict_of_imgs,
        figsize = (6,6),
        cmap    = None,
        OUT_DIR = "",
        tag     = "",
        show    = False
    ):
    fig = plt.figure(figsize=figsize)
    sqr = int(np.ceil(np.sqrt(len(dict_of_imgs))))

    for i,label in enumerate(dict_of_imgs):
        ax = plt.subplot(sqr,sqr,i+1)
        ax.imshow(dict_of_imgs[label], cmap=cmap)
        plt.xlabel(label)

    plt.tight_layout()
    if OUT_DIR:
        fig.savefig("{}/plot_{}.png".format(OUT_DIR, tag), bbox_inches = 'tight')
    if not show:
        plt.close(fig)
    return fig


#=======================#
#  D E F I N I T I O N  #
#=======================#
# class Job(threading.Thread):
#     def __init__(self, interval, execute, *args, **kwargs):
#         threading.Thread.__init__(self)
#         self.daemon = False
#         self.stopped = threading.Event()
#         self.interval = interval
#         self.execute = execute
#         self.args = args
#         self.kwargs = kwargs
        
#     def stop(self):
#                 self.stopped.set()
#                 self.join()
#     def run(self):
#             while not self.stopped.wait(self.interval.total_seconds()):
#                 self.execute(*self.args, **self.kwargs)

# class Job_Engine:
#     def _signal_handler(self, signum, frame):
#         raise JOB_InterruptException
    
#     def __init__(self, rate_Hz, name, update_function):
#         self._job = Job(interval=timedelta(seconds=1/rate_Hz), execute=update_function)
#         self._name = name

#     def start(self):
#         self._job.start()

class Engine_InterruptException(Exception):
    pass


class Mujoco_Engine:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _camera_config = {
        "camera/zed/L": {"width": 1280, "height":720, "fps": 60},
        "camera/zed/R": {"width": 1280, "height":720, "fps": 60},
    }
    _camera_views = {}
    _IC_state = None
    _core = None
    
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self,  xml_path, rate_Hz, camera_config=None, name="DEFAULT"):
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        ## Init Configs:
        if camera_config:
            self._camera_config.update(camera_config)
        self._name = name
        self._rate_Hz = rate_Hz
        
        ## Initiate MJ
        self.mj_model = mujoco.MjModel.from_xml_path(xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        
        ## MJ Viewer:
        self.mj_viewer = mujoco_viewer.MujocoViewer(self.mj_model, self.mj_data, width=800, height=800, title="main")
        # if len(self._camera_config):
        # self.mj_viewer_off = mujoco_viewer.MujocoViewer(self.mj_model, self.mj_data, width=800, height=800, title="camera-view")
        self._t_update = time.time()
        
    #==================================#
    #  P U B L I C    F U N C T I O N  #
    #==================================#
    def shutdown(self):
        self.mj_viewer.close()
        if self.mj_viewer_off:
            self.mj_viewer_off.close()
            
    def is_shutdown(self):
        try:
            return False
        except Engine_InterruptException:
            print("[Job_Engine::{}] Program killed: running cleanup code".format(self._name))
            self.shutdown()
            return True
    
    #====================================#
    #  P R I V A T E    F U N C T I O N  #
    #====================================#    
    def _signal_handler(self, signum, frame):
        raise Engine_InterruptException
    
    def _internal_engine_update(self):
        self._update()

    def _update(self):
        delta_t = time.time() - self._t_update
        print("FPS: {0}".format(1/delta_t))
        # - command joint control angles:
        self.mj_data.actuator("wam/J1/P").ctrl = -1.92
        self.mj_data.actuator("wam/J2/P").ctrl = 1.88

        # - render current view:
        mujoco.mj_step(self.mj_model, self.mj_data)
        self.mj_viewer.render()
        # self.mj_viewer_off.render()

        # - capture view:
        camera_sensor_data = self.mj_viewer.acquire_sensor_camera_frames()
        # print(camera_sensor_data["frame_stamp"])
        # imgs_plot(dict_of_imgs=camera_sensor_data["frame_buffer"], figsize=(6,2), OUT_DIR="output", tag="test")
        
        # - update:
        self._t_update = time.time()
        
    
