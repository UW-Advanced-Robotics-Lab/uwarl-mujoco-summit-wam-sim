# %% Import
import gym
import mujoco_py



# %% Simple Viewer
def test_viewer(xml_path):
    model = mujoco_py.load_model_from_path(xml_path)
    sim = mujoco_py.MjSim(model)
    viewer = mujoco_py.MjViewer(sim)
    for _ in range(100):
        sim.step()
        viewer.render()


test_viewer(xml_path="../playground/playground_mobile_grasping.xml")
