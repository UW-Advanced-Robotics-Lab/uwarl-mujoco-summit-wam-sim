# simulation-mujoco-summit-wam
Mujoco Physics Simulation Package for Waterloo steel robot

### Custom Library to support with MuJoCo 2.2.x
- Launch Package (includes models) (**This Package**)
- Graphical User Interface / Direct OnOff-screen Render: [jx-mujoco-python-viewer](https://github.com/jaku-jaku/jx-mujoco-python-viewer)
- Main Engine Code [jx-mujoco-python-engine](https://github.com/jaku-jaku/jx-mujoco-python-engine) (similar to [deepmind/dm_control](https://github.com/deepmind/dm_control))
- MuJoCo 2.2.x locking variants [jx-mujoco](https://github.com/jaku-jaku/jx-mujoco)
- Unity Project for MuJoCo-Unity-Integration [simulation-mujoco-summit-wam-unity](https://github.com/UW-Advanced-Robotics-Lab/simulation-mujoco-summit-wam-unity)

## Preview:
### Waterloo Steel Mobile Playground (engine & viewer):
<img src="./documentation/main.png" alt="waterloo_steel" height="600"/>

## ToDo:
- [x] Full Assembly
- [x] Simulation setup
- [x] Contact Physics [Last Edit: 15/Jun/2022]
- [x] [WAM] Ensure Mechanical Params are Verified
- [x] [BHAND] Ensure Mechanical Params are Verified
- [?] [SUMMIT] Ensure Mechanical Params are Verified
- [x] Control Descriptors
- [ ] Example Interfacing code
- [ ] ....

## Note:
> :announcement: Starting version 2.1.2, MuJoCo comes with python bindings, no need to look into mujoco_py package (which only works for 210)
> Migration notes: https://mujoco.readthedocs.io/en/latest/python.html#migration-notes-for-mujoco-py
> Right now, we will use mujoco-viewer based on https://github.com/rohanpsingh/mujoco-python-viewer
> TODO: We will migrate to the official viewer in python bindings later: Watch PR: https://github.com/deepmind/mujoco/pull/201
## File Hierarchy:
```
.
├── CITATION.cff
├── LICENSE
├── README.md
├── components
│   ├── include_common.xml
│   ├── include_{assembly-name}_Chain.xml
│   ├── include_{assembly-name}_Dependencies.xml
│   ├── include_{assembly-name}_actuators.xml
│   └── ...
├── documentation
│   └── ...
├── meshes
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
│   ├── jx-mujoco-python-viewer # [Mujoco Render/Interaction GUI]
│   └── jx-mujoco-python-engine # [Main engine code]
└── textures
│   └── ...
x

[ 5 directories, # files ]
```

## Appendix:
### Note:
- WAM sim file is a CORRECTED and MODIFIED version based on [the official archived MuJoCo model made by Vikash kumar](https://roboti.us/forum/index.php?resources/wam-and-barrett-hand.20/)
    - Findings: The original model has collision disabled, and parameters are incorrectly populated
    - Note: We have modified the original model based on the given stl files completely, and configured MoI based on [the Official Barrett WAM Specification](https://web.barrett.com/support/WAM_Documentation/WAM_InertialSpecifications_AC-02.pdf). 
        - Specifically, we made exactly the same as described in the document, and removed incorrect quaternion parameters for `inertial` , and populated the `inertial` purely based on the centre of the mass and translated the coordinate frames to the stl model frame (manually)
- Shall you have any concern with the parameters, kindly open an issue.

### Waterloo Steel Mobile Manipulator Simulation:
#### Launch Demo:

#### Joints and MOI:
Joints             |  MOI
:-------------------------:|:-------------------------:
<img src="./documentation/joints.png" alt="waterloo_steel" height="300"/>  |  <img src="./documentation/MoI.png" alt="waterloo_steel" height="300"/>


### Installation Guide
- Install MuJoCo 2.2.x via `$ sudo pip install mujoco`
- Download MuJoCo 2.2.x release package from https://github.com/deepmind/mujoco/releases
#### Tested Platforms:
- M1 Macbook Pro 14" 
- Ubuntu 20.04

#### M1 Macbook:
- Make directory `MuJoCo_v2.2` under `/Applications`
- Copy all files from **MuJoCo 2.2.x** release dmg into `/Applications/MuJoCo_v2.2`

#### Unity module:
- The directory for the `.dylib` has been modified to `/Applications/MuJoCo_v2.2/MuJoCo.app/Contents/Frameworks` under this `submodules/jx-mujoco`

#### Submodules:
1. Submodule Update
    ```zsh
    $ cd submodules
    $ git submodule update
    ```
2. Install editable [python viewer](https://github.com/jaku-jaku/jx-mujoco-python-viewer):
    ```zsh
    $ cd jx-mujoco-python-viewer
    $ pip install -e .
    ```
3. Install editable [python engine](https://github.com/jaku-jaku/jx-mujoco-python-engine):
    ```zsh
    $ cd jx-mujoco-python-engine
    $ pip install -e .
    ```

#### Launch:
##### Cart Manipulation:
```
$ cd src && python main.py
```
