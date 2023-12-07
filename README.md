<toc>

# Table of Contents
[*Last generated: Wed 13 Sep 2023 10:56:48 EDT*]
- [**Mujoco Simulation Packages**](#Mujoco-Simulation-Packages)
  - [1. Preview:](#1-Preview)
    - [1.1 Custom Library to support with MuJoCo 2.2.x](#11-Custom-Library-to-support-with-MuJoCo-22x)
    - [1.2 Waterloo Steel Mobile Playground (engine & viewer):](#12-Waterloo-Steel-Mobile-Playground-engine-viewer)
  - [2. ToDo:](#2-ToDo)
  - [3. Note:](#3-Note)
  - [4. File Hierarchy:](#4-File-Hierarchy)
  - [A*. Appendix:](#A-Appendix)
    - [A.1 Note:](#A1-Note)
    - [A.2 Waterloo Steel Mobile Manipulator Simulation:](#A2-Waterloo-Steel-Mobile-Manipulator-Simulation)
      - [A.2.1 AJoints and MOI:](#A21-AJoints-and-MOI)
    - [A.2.2 Installation Guide](#A22-Installation-Guide)
      - [A.2.3 Tested Platforms:](#A23-Tested-Platforms)
      - [A.2.4 M1 Macbook Instructions:](#A24-M1-Macbook-Instructions)
      - [A.2.5 Unity module:](#A25-Unity-module)
      - [A.2.6 Submodules:](#A26-Submodules)
      - [A.2.7 Launch:](#A27-Launch)
        - [Cart Manipulation:](#Cart-Manipulation)

---
</toc>
# Mujoco Simulation Packages

Mujoco Physics Simulation Package for Waterloo Steel Robot

## 1. Preview:

### 1.1 Custom Library to support with MuJoCo 2.2.x

- [x] Launch Package (includes models) (**This Package**)
- [x] Graphical User Interface / Direct OnOff-screen Render: [uwarl-mujoco-python-viewer](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-viewer)
  -  branching from [jx-mujoco-python-viewer](https://github.com/jaku-jaku/jx-mujoco-python-viewer)
- [x] Main Engine Code [uwarl-mujoco-python-engine](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-engine)
  - branching from [jx-mujoco-python-engine](https://github.com/jaku-jaku/jx-mujoco-python-engine) (similar to [deepmind/dm_control](https://github.com/deepmind/dm_control))
- [x] MuJoCo 2.2.x locking variants [jx-mujoco](https://github.com/jaku-jaku/jx-mujoco)
- [x] [TODO: ROS Integration] for bridging SIL and HIL
- [ ] [TODO: Unity Integration] for rendering and realistic camera views
- [ ] [TODO: Migration to MuJoCo 3] for rendering higher simulation speeds

### 1.2 Waterloo Steel Mobile Playground (engine & viewer):
<img src="./documentation/main.png" alt="waterloo_steel" height="600"/>

## 2. ToDo:
- [x] Full Assembly
- [x] Simulation setup
- [x] Contact Physics [Last Edit: 15/Jun/2022]
- [x] [WAM] Ensure Mechanical Params are Verified
- [x] [BHAND] Ensure Mechanical Params are Verified
- [?] [SUMMIT] Ensure Mechanical Params are Verified
- [x] Control Descriptors
- [x] PID control for base
- [x] ROS integration
- [x] Real-time simulation synchronization
- [ ] Find solution for convex hull of wagon handle
- [ ] Passing all variables in highest level launch file
- [ ] ....

## 3. Note:
> :announcement: Starting version 2.1.2, MuJoCo comes with python bindings, no need to look into mujoco_py package (which only works for 210)
> Migration notes: https://mujoco.readthedocs.io/en/latest/python.html#migration-notes-for-mujoco-py
> Right now, we will use mujoco-viewer based on https://github.com/rohanpsingh/mujoco-python-viewer
> TODO: We will migrate to the official viewer in python bindings later: Watch PR: https://github.com/deepmind/mujoco/pull/201
## 4. File Hierarchy:
```
.
├── CITATION.cff
├── LICENSE
├── README.md
├── components
│   ├── meshes
│   │   ├── bases
│   │   ├── meshes_bhand
│   │   └── ...
│   ├── robots
│   │   ├── wam_7dof_wam_bhand.urdf.xacro
│   │   └── waterloo_steel_mujoco.urdf.xacro
│   ├── urdf
│   │   ├── bases
│   │   ├── wagon
│   │   ├── wam
│   │   └── wheels
│   ├── include_common.xml
│   ├── include_{assembly-name}_Chain.xml
│   ├── include_{assembly-name}_Dependencies.xml
│   ├── include_{assembly-name}_actuators.xml
│   └── ...
├── documentation
│   └── ...
├── include
│   └── mujoco_ros_control
│   │   ├── mujoco_ros_control.h
│   │   ├── robot_hw_sim_plugin.h
│   │   └── robot_hw_sim.h
├── meshes
│   ├── maps_thirdfloor
│   │   ├── map_e7_3_v6.stl
│   │   └── ...
│   ├── meshes_{module-name}
│   │   ├── {3D-model-component-name}.stl
│   │   └── ...
│   └── ...
├── playground
│   ├── playground_{playground-name}.xml
│   └── ...
├── src
│   ├── {scripts}.py # [launch files]
│   └── ...
├── submodules
│   ├── uwarl-mujoco-python-viewer # [Mujoco Render/Interaction GUI]
│   └── uwarl-mujoco-python-engine # [Main engine code]
└── textures
│   └── ...
x

[ 5 directories, # files ]
```

## A*. Appendix:
### A.1 Note:
- WAM sim file is a CORRECTED and MODIFIED version based on [the official archived MuJoCo model made by Vikash kumar](https://roboti.us/forum/index.php?resources/wam-and-barrett-hand.20/)
    - Findings: The original model has collision disabled, and parameters are incorrectly populated
    - Note: We have modified the original model based on the given stl files completely, and configured MoI based on [the Official Barrett WAM Specification](https://web.barrett.com/support/WAM_Documentation/WAM_InertialSpecifications_AC-02.pdf). 
        - Specifically, we made exactly the same as described in the document, and removed incorrect quaternion parameters for `inertial` , and populated the `inertial` purely based on the centre of the mass and translated the coordinate frames to the stl model frame (manually)
- Shall you have any concern with the parameters, kindly open an issue.

### A.2 Waterloo Steel Mobile Manipulator Simulation:

#### A.2.1 AJoints and MOI:
Joints             |  MOI
:-------------------------:|:-------------------------:
<img src="./documentation/joints.png" alt="waterloo_steel" height="300"/>  |  <img src="./documentation/MoI.png" alt="waterloo_steel" height="300"/>

### A.2.2 Installation Guide
- Install MuJoCo 2.2.x via `$ sudo pip install mujoco`
- Download MuJoCo 2.2.x release package from https://github.com/deepmind/mujoco/releases
#### A.2.3 Tested Platforms:
- [x] M1 Macbook Pro 14" 
- [x] Ubuntu 20.04
- [ ] [TBD] WINDOWS ---x

#### A.2.4 M1 Macbook Instructions:

- Make directory `MuJoCo_v2.2` under `/Applications`
- Copy all files from **MuJoCo 2.2.x** release dmg into `/Applications/MuJoCo_v2.2`

#### A.2.5 Unity module:
- The directory for the `.dylib` has been modified to `/Applications/MuJoCo_v2.2/MuJoCo.app/Contents/Frameworks` under this `submodules/jx-mujoco`

#### A.2.6 Submodules:
1. Submodule Update
    ```zsh
    $ cd submodules
    $ git submodule update
    ```
2. Install editable [python viewer](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-viewer):
    ```zsh
    $ cd uwarl-mujoco-python-viewer
    $ pip install -e .
    ```
3. Install editable [python engine](https://github.com/UW-Advanced-Robotics-Lab/uwarl-mujoco-python-engine):
    ```zsh
    $ cd uwarl-mujoco-python-engine
    $ pip install -e .
    ```

### A.3 Launch:
#### A.3.1 Cart Manipulation:
Launches the cart manipulation simulation with ROS integration. The launch file will launch:
- Clock publisher for simulation time
- Mujoco Engine and Viewer based on submodules
- ROS controllers which are used for trajectory control of the WAM
- Hardware simulation interface node to connect MuJoCo to ROS controllers
- Trajectory following action server for mobile base 
- Rosbag recorder action server
- `Demo_V09_mujoco.py` node 

```
$ roslaunch waterloo_steel_sim_bringup waterloo_steel_complete_cart_mujoco.launch
```

##### Variables of simulation:
These variables can be changed depending on the simulation. 
- In `launch/mujocolaunch.launch` param `sim_frequency_mujoco`. This changes the frequency of the node which updates the engine. It is synched to the simulation Hardware Simulation Interface for control of the WAM.
- In `components/include_common.xml` param `timestep` for changing the engines stepsize.

> [!IMPORTANT]
> Make sure 1/frequency of the ROS node updating the engine is equal to a multiple of the engine stepsize for real-time simulation.

- In `playground/playground_mobile_wagon_manipulation.xml` comment out the world body and the contact exclusions to simulate without a world. This increases the rendering performance. 
  - You might want to compress the mesh to increase rendering performance, by using MeshLab for example. 
- In `components/include_e7_3rd_floor_Dependencies.xml`, the world .stl file is defined. Change to simulate different world.
- In `src/main.py` the MuJoCo viewer rate can be defined. 
- In `src/main.py` the onboard cameras of the Summit and WAM can be enabled. This can be done by passing `True` in the Engine `_update` function. Default value equals `False`.


### A.4 MuJoCo-ROS integration
<img src="./documentation/mujoco_ros_integration.png" alt="mujoco_ros_integration" height="300"/>

<img src="./documentation/mujoco_ros_integration_detailed.png" alt="mujoco_ros_integration_detailed" height="300"/>

To find more details on the ROS integration of the MuJoCo simulator, see the pdf file in the documentation folder.

<eof>

---
[*> Back To Top <*](#Table-of-Contents)
</eof>