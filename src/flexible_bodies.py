import mujoco
import mediapy as media

xml = """
<mujoco model="playground_wagon_manipulation(v1.0.0)">
    <!--#########################
        ### Compiler Specific ###
        ######################### -->
    <compiler coordinate="local" angle="radian"/>
    
    <!-- Options: -->
    <option 
        timestep="1e-3"
        iterations="10"
        tolerance="1e-10"
        impratio="1"
        gravity="0 0 -9.81"
        solver="Newton"
        cone="elliptic" 
    />
    <default>
        <geom margin="0.001" solimp="0.99 0.99 0.01" solref="0.01 1" condim="4" /> 
    </default>
    <!-- Statistic -->
    <!-- https://mujoco.readthedocs.io/en/latest/XMLreference.html#statistic -->
    <!-- Looka at this `center` point using the free-camera, which is located at a distance of 1.5x`extent` distance -->
    <!-- Apparently, the camera cannot zoom to within the extent distance, so keep it small. -->
    <!-- The value of `center` does not move the initial location of the camera view-point.-->
    <statistic center="0 0 0" extent="1"/>
    <!-- Visual -->
    <visual>
        <map fogstart="3" fogend="5" force="0.1" znear="0.001"/>
        <quality shadowsize="1024" offsamples="16"/>
        <scale forcewidth="0.05" contactwidth="0.05" contactheight="0.05" 
               connect="0.05" com="0.1" jointlength="0.1" jointwidth="0.01"
               framelength="0.1" framewidth="0.01"/>
        <!-- <global offwidth="1960" offheight="800"/> -->
    </visual>

    <!--############################
        ### Environment Specific ###
        ############################ -->
    <asset>
        <!-- [ENV] Ground PLane -->
        <texture name="groundplane" type="2d" builtin="checker" rgb1=".25 .26 .25" 
            rgb2=".22 .22 .22" width="100" height="100" mark="edge" markrgb=".3 .3 .3"/>
        <material name="MatGnd" texture="groundplane"  texrepeat="5 5" specular="1" shininess=".001" reflectance="0.00001"/>
    </asset>

    <!--##################
        ### Extension: ###
        ################## -->
    <extension>
        <plugin plugin="mujoco.elasticity.solid"/>
    </extension>

    <!--###############
        ### Config: ###
        ############### -->
    <!-- DEFAULT -->
    <default>
        <!-- The class-name being specified under default maps back to the class-names being specified for a body (childclass/class) , or  -->
        <!-- These names have to be unique; it must not clash with class-names for un-intended bodies/joints/geometries -->
        <default class="summit">
        </default>
        <default class="summit/whl">
            <!-- class-names for joints -->
            <joint axis="1 0 0" damping="0.55"/>
            <!-- Bydefault, hinge-type is the default joint type -->
            <!-- https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint -->
        </default>
        <default class="summit/col">
            <!-- class-names for geometry-type -->
            <geom contype="1" conaffinity="1" group="0" rgba="0.5 0.6 0.7 0.3"/>
        </default>
        <default class="summit/body/viz">
            <geom contype="0" conaffinity="0" group="1" rgba="0.6 0.6 0.6 1"/>
        </default>
        <default class="summit/wheel/viz">
            <geom contype="0" conaffinity="0" group="1" rgba="0.3 0.3 0.3 1"/>
        </default>
        <default class="summit/camera/viz">
            <geom contype="0" conaffinity="0" group="1" rgba="0.7 0.7 0.7 1"/>
        </default>

        <default class="wam">
        </default>
    </default>

    <worldbody>
        <!-- GND -->
        <light directional="true" cutoff="60" exponent="1" diffuse="1 1 1" specular=".1 .1 .1" pos=".1 .2 1.3" dir="-.1 -.2 -1.3"/>
        <geom name="ground" type="plane" pos="0 0 0" size="24 24 1" conaffinity="1" contype="1" material="MatGnd" friction="0.0001 0.001 0.001" group="1"/>

        <body name="smt/base_link" childclass="summit" pos="0 0 0">
            <!-- === BASE LINK === === === === -->
            <!-- mobile body -->
            <inertial                                           pos="0 0 0.37" mass="125" diaginertia="1.391 6.853 6.125" />
            <geom name="smt/viz/base_link"                      class="summit/body/viz" pos="0 0 0.42" size="0.24 0.32 0.16" type="box"/>

            <!-- Front Camera -->
            <body name="smt/front/camera" pos="0 0.362 0.373" quat="0 0 0.707107 0.707107">
                <inertial               pos="0 0 0.015" mass="0.095"/>
                <geom                   class="summit/camera/viz" size="0.06 0.04" type="cylinder"/>
                <camera name="smt/front/camera/intel/rgb" pos="0 0 0" quat="0 0 1 0" fovy="55"/>
            </body>
            <!-- Rear Camera --><!-- quant=w x y z -->
            <body name="smt/rear/camera" pos="0 -0.362 0.373" quat="0.707107 0.707107 0 0"> 
                <inertial               pos="0 0 0.015" mass="0.095"/>
                <geom                   class="summit/camera/viz" size="0.06 0.04" type="cylinder"/>
                <camera name="smt/rear/camera/intel/rgb" pos="0 0 0" quat="0 0 1 0" fovy="55"/>
            </body>
            <flexcomp name="softbody_1" type="grid" count="4 4 4" spacing=".01 .01 .01" pos="0 0.32 1" radius=".0" rgba="0 0 0.7 1" dim="3" mass="1">
                <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
                <edge damping="1"/>
                <plugin plugin="mujoco.elasticity.solid">
                    <config key="poisson" value="0.2"/>
                    <config key="young" value="5e4"/>
                </plugin>
                <!-- Without below line, we get 'rank-deficiet matrix' -->
                <pin id="0 1 2 3 16 17 18 19 32 33 34 35 48 49 50 51"/>
            </flexcomp>
            <!-- Simplified the collision into boxes + arm rest -->
            <geom name="smt/col/base_link/box_body"             class="summit/col" pos="0 0 0.42" size="0.24 0.32 0.16" type="box"/>
            <geom name="smt/col/base_link/cylinder_lidar"       class="summit/col" pos="0.246 0.362 0.313" size="0.06 0.044" type="cylinder"/>

            <!-- world placement -->
            <joint name="smt/world_x" armature="0.0001" axis="0 1 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
            <joint name="smt/world_y" armature="0.0001" axis="-1 0 0" damping="1e+11" pos="0 0 0" type="slide"></joint>
            <joint name="smt/world_z" armature="0.0001" axis="0 0 1" damping="1e+0"  pos="0 0 0" type="slide"></joint>

            <!-- control summit base -->
            <joint name="smt/pose/x"                            type="slide" pos="0 0 0.4" axis="0 1 0"  damping="15"/>
            <joint name="smt/pose/y"                            type="slide" pos="0 0 0.4" axis="-1 0 0"  damping="15"/>
            <joint name="smt/orie/z"                            type="hinge" pos="0 0 0.4" axis="0 0 1"  damping="10"/>

            <!-- === whl LINK === === === === -->
            <!-- Mobile whls -->
            <body name="smt/whl/LF_link" pos="0 0 0">
                <geom name="smt/viz/whl/LF"                    class="summit/wheel/viz" pos="-0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder"/>
                <geom name="smt/col/whl/LF_cylinder"           class="summit/col" pos="-0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                <inertial                                                         pos="-0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                <joint name="smt/whl_LF"                       class="summit/whl" pos="-0.264 0.222 0.128"/>
            </body>
            <body name="smt/whl/LR_link" pos="0 0 0">
                <geom name="smt/viz/whl/LR"                    class="summit/wheel/viz" pos="-0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder"/>
                <geom name="smt/col/whl/LR_cylinder"           class="summit/col" pos="-0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                <inertial                                                         pos="-0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                <joint name="smt/whl_LR"                       class="summit/whl" pos="-0.264 -0.223 0.128"/>
            </body>
            <body name="smt/whl/RF_link" pos="0 0 0">
                <geom name="smt/viz/whl/RF"                   class="summit/wheel/viz" pos="0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder"/>
                <geom name="smt/col/whl/RF_cylinder"          class="summit/col" pos="0.220 0.222 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                <inertial                                                        pos="0.220 0.222 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                <joint name="smt/whl_RF"                      class="summit/whl" pos="0.264 0.222 0.128"/>
            </body>
            <body name="smt/whl/RR_link" pos="0 0 0">
                <geom name="smt/viz/whl/RR"                   class="summit/wheel/viz" pos="0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder"/>
                <geom name="smt/col/whl/RR_cylinder"          class="summit/col" pos="0.220 -0.223 0.128" quat="0.707107 0 0.707106 0" size="0.13 0.05" type="cylinder" friction="0.0001 0.001 0.001"/>
                <inertial                                                        pos="0.220 -0.223 0.128" mass="10.0" diaginertia="0.04411 0.02467 0.02467"/>
                <joint name="smt/whl_RR"                      class="summit/whl" pos="0.264 -0.223 0.128"/>
            </body>

            <!-- === WAM 7DOF HAND LINK === === === === -->
            <!-- Place WAM Module Here -->
            <!-- WAM -->
            <body name="wam/base_link" childclass="wam" pos="0 0 0">
                <body name="wam/base" childclass="wam" pos="0 0 0">
                    <inertial   pos="-0.14071720 -0.02017671 0.07995294" mass="9.97059584" fullinertia="0.11760385 0.10916849 0.18294303 0.02557874 0.00161433 0.00640270" /> 
                    <geom name="cylinder_0/col" type="cylinder" size=".1 .1" contype="1" conaffinity="1" group="0" rgba=".8 .2 .2 1"/>
                    <geom name="cylinder_0/viz" type="cylinder" size=".1 .1" contype="0" conaffinity="0" group="1" rgba=".2 .2 .2 1"/>
                    <!-- <flexcomp name="softbody_2" type="grid" count="4 4 4" spacing=".01 .01 .01" pos="0 0.3 0.4" radius=".0" rgba="0 0 0.7 1" dim="3" mass="1">
                        <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
                        <edge damping="1"/>
                        <plugin plugin="mujoco.elasticity.solid">
                            <config key="poisson" value="0.2"/>
                            <config key="young" value="5e4"/>
                        </plugin>
                        <pin id="0 1 2 3 16 17 18 19 32 33 34 35 48 49 50 51"/>
                    </flexcomp> -->
                </body>
            </body>
        </body>
    </worldbody>

</mujoco>
"""
# model = mujoco.MjModel.from_xml_string(xml)
model = mujoco.MjModel.from_xml_path("/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_one_handed_cart_exp.xml")
data = mujoco.MjData(model)
try:
  model.geom()
except KeyError as e:
  print(e)
# enable joint visualization option:
scene_option = mujoco.MjvOption()
scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

duration = 3.8  # (seconds)
framerate = 60  # (Hz)

# Simulate and display video.
frames = []
mujoco.mj_resetData(model, data)
with mujoco.Renderer(model) as renderer:
  while data.time < duration:
    mujoco.mj_step(model, data)
    if len(frames) < data.time * framerate:
      renderer.update_scene(data, scene_option=scene_option)
      pixels = renderer.render()
      frames.append(pixels)

print("Hey!")
media.show_video(frames, fps=framerate)