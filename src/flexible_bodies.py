import mujoco
import mediapy as media

xml = """
<mujoco>
  <option timestep=".001"/>
  <extension>
    <plugin plugin="mujoco.elasticity.solid"/>
  </extension>
  <asset>
    <!-- https://unsplash.com/photos/black-textile-on-brown-wooden-table-BUDn0s7QteI -->
        <texture name="concrete_floor" file="/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/components/texture/the-creativv-BUDn0s7QteI-unsplash.png" type="2d"/>
        <material name="floor" texture="concrete_floor" reflectance="0.05"/>
        <!-- https://www.deviantart.com/ebonykate/art/Free-textures-23-85711516 -->
        <texture name="blue_sponge" file="/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/components/texture/blue_sponge.png" type="2d"/>
        <material name="blue_sponge" texture="blue_sponge" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light directional="true" cutoff="60" exponent="1" diffuse="1 1 1" specular=".1 .1 .1" pos=".1 .2 1.3" dir="-.1 -.2 -1.3"/>
    <geom name="ground" type="plane" pos="0 0 0" size="24 24 1" conaffinity="1" contype="1" material="floor" friction="0.0001 0.001 0.001" group="1"/>
    <flexcomp name="softbody" type="grid" count="24 4 4" spacing=".1 .1 .1" pos=".1 0 1.5" radius=".0" rgba="0 .7 .7 1" dim="3" mass="7">
      <contact condim="3" solref="0.01 1" solimp=".95 .99 .0001" selfcollide="none"/>
      <edge damping="1"/>
      <plugin plugin="mujoco.elasticity.solid">
          <config key="poisson" value="0.2"/>
          <config key="young" value="5e4"/>
      </plugin>
    </flexcomp>
  </worldbody>
</mujoco>
"""
# model = mujoco.MjModel.from_xml_string(xml)
model = mujoco.MjModel.from_xml_path('/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/playground/playground_one_handed_cart_exp_1.xml')
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