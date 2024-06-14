import os

from dm_control import mjcf
from dm_control.mujoco.wrapper import util

# Make a class for constructing the cameras
class Camera:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _com_pos = (0, 0, 0.015)
    _dclass = 'wam\\viz'
    _mass = 0.095
    _mesh_name = 'intel_realsense_l515'
    _name = '\\rgb'
    _quat = (0, 0, 1, 0)
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name, fovy):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element: No need to add position info, as that will be added based on which `site` this body is going to be attached (https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md#attachment-frames)
        self.camera_body = self.mjcf_model.worldbody.add('body',
                                                         name = name)
        # Add Inertial properties
        self.camera_body.add('inertial',
                             pos = self._com_pos, 
                             mass = self._mass)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.camera_body.add('geom',
                             mesh = self._mesh_name,
                             dclass = self._dclass)
        # Add camera sensor
        self.camera_body.add('camera',
                             name = name+self._name,
                             pos = (0, 0, 0),
                             quat = self._quat,
                             fovy = fovy)

# Make a class for constructing a link with joints
class Link_w_Joints:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _dclass_col = 'wam\\col'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, col_meshes, com_pos, inertia, joint_damping, joint_frictionloss, joint_name, joint_range, link_pos, mass, name, viz_meshes):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.link_body = self.mjcf_model.worldbody.add('body',
                                                       name = name,
                                                       pos = link_pos)
        # Add Inertial properties
        self.link_body.add('inertial',
                           pos = com_pos, 
                           mass = mass,
                           fullinertia = inertia)
        # Add Joints
        self.link_body.add('joint',
                           name = joint_name,
                           range = joint_range, 
                           damping = joint_damping,
                           frictionloss = joint_frictionloss)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        # For visualization
        self.link_body.add('geom',
                           dclass = name+'\\viz',
                           mesh = viz_meshes)
        # For Collision
        for x in range(len(col_meshes)):
            self.link_body.add('geom',
                               dclass = self._dclass_col,
                               mesh = col_meshes[x])

# Make a class for constructing a link without joints
class Link_wo_Joints:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _dclass_col = 'wam\\col'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, col_meshes, com_pos, inertia, link_pos, mass, name, viz_meshes):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.link_body = self.mjcf_model.worldbody.add('body',
                                                       name = name,
                                                       pos = link_pos)
        # Add Inertial properties
        self.link_body.add('inertial',
                           pos = com_pos, 
                           mass = mass,
                           fullinertia = inertia)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        # For visualization
        self.link_body.add('geom',
                           dclass = name+'\\viz',
                           mesh = viz_meshes)
        # For Collision
        for x in range(len(col_meshes)):
            self.link_body.add('geom',
                               dclass = self._dclass_col,
                               mesh = col_meshes[x])

# Make a class for constructing the WAM
class WAM:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _body_name = ['base_link',
                  'base',
                  'shoulder_yaw_link',
                  'shoulder_pitch_link',
                  'upper_arm_link',
                  'forearm_link',
                  'wrist_yaw_link',
                  'wrist_pitch_link',
                  'wrist_palm_link',
                  'torque_sensor_link']
    _dclass_col = 'wam\\col'
    _col_mesh_names = [['base_link_convex'],
                       ['shoulder_link_convex_decomposition_p1',
                        'shoulder_link_convex_decomposition_p2',
                        'shoulder_link_convex_decomposition_p3'],
                       ['shoulder_pitch_link_convex'],
                       ['upper_arm_link_convex_decomposition_p1',
                        'upper_arm_link_convex_decomposition_p2'],
                       ['elbow_link_convex'],
                       ['forearm_link_convex_decomposition_p1',
                        'forearm_link_convex_decomposition_p2'],
                       ['wrist_yaw_link_convex_decomposition_p1',
                        'wrist_yaw_link_convex_decomposition_p2'],
                       ['wrist_pitch_link_convex_decomposition_p1',
                        'wrist_pitch_link_convex_decomposition_p2',
                        'wrist_pitch_link_convex_decomposition_p3'],
                       ['wrist_palm_link_convex']]
    _com_pos = [[-0.14071720, -0.02017671, 0.07995294],
                [-0.00443422, -0.00066489, -0.12189039],
                [-0.00236983, -0.01542114, 0.03105614],
                [0.00674142, -0.00003309, 0.3424923],
                [-0.04001488, -0.1327162, -0.00022942],
                [0.00008921, 0.00435824, -0.00511217],
                [-0.00012262, -0.02468336, -0.01703194],
                [-0.00007974, 0.0001613, 0.05176448],
                [0, 0.0085, 0]]
    _inertial = [[0.11760385, 0.10916849, 0.18294303, 0.02557874, 0.00161433, 0.00640270],
                 [0.13488033, 0.09046330, 0.11328369, -0.00012485, 0.00213041, -0.00068555],
                 [0.02140958, 0.01558906, 0.01377875, -0.00002461, 0.00027172, 0.00181920],
                 [0.05911077, 0.05927043, 0.00324550, -0.00000738, 0.00249612, -0.00001767],
                 [0.01491672, 0.00294463, 0.01482922, 0.00150604, 0.00001741, 0.00002109],
                 [0.00005029, 0.00006270, 0.00007582, -0.00000005, -0.00000020, 0.00000359],
                 [0.00055516, 0.00045358, 0.00024367, 0.00000074, 0.00000061, 0.00004590],
                 [0.00003773, 0.00003806, 0.00007408, -0.00000019, 0.00000000, 0.00000000],
                 [0.00007551, 0.0001462, 0.00007508, -7.002e-8, -1.199e-7, -5.37e-8]]
    _joint_damping = [1000,
                      1000,
                      500,
                      100,
                      50,
                      50,
                      10]
    _joint_frictionloss = 1000
    _joint_name = ['wam/J1',
                   'wam/J2',
                   'wam/J3',
                   'wam/J4',
                   'wam/J5',
                   'wam/J6',
                   'wam/J7']
    _joint_range = [[-2.6, 2.6],
                    [-2.0, 2.0],
                    [-2.8, 2.8],
                    [-0.9, 3.1],
                    [-4.8, 1.3],
                    [-1.6, 1.6],
                    [-2.2, 2.2]]
    _mass = [9.97059584,
             10.76768767,
             3.8749,
             1.8028141,
             2.40016804,
             0.12376019,
             0.41797364,
             0.06864753,
             0.133278]
    _pos = [[0, 0, 0],
            [0, 0, 0.346],
            [0, 0, 0],
            [0, 0, 0],
            [0.045, 0, 0.55],
            [-0.045, -0.3, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0.06]]
    _viz_mesh_names = ['base_link_fine',
                       'shoulder_link_fine',
                       'shoulder_pitch_link_fine',
                       'upper_arm_link_fine',
                       'elbow_link_fine',
                       'forearm_link_fine',
                       'wrist_yaw_link_fine',
                       'wrist_pitch_link_fine',
                       'wrist_palm_link_fine',
                       'torque_sensor_plate']
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.summit_body = self.mjcf_model.worldbody.add('body',
                                                         name = prefix+'\\'+wheel_name+'_link',
                                                         pos = (0, 0, 0))


# Make a class for constructing the cameras
class Wheel:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _dclass_col = 'summit\\col'
    _dclass_joint = 'summit\\whl'
    _dclass_vis = 'summit\\wheel\\viz'
    _friction = (0.0001, 0.001, 0.001)
    _inertia = (0.04411, 0.02467, 0.02467)
    _mass = 10
    _size = (0.13, 0.05)
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, mesh, prefix, wheel_name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=prefix+'\\'+wheel_name+'_link')
        # Add a body element:
        self.summit_body = self.mjcf_model.worldbody.add('body',
                                                         name = prefix+'\\'+wheel_name+'_link',
                                                         pos = (0, 0, 0))
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.summit_body.add('geom',
                             name = prefix+'\\viz\\'+wheel_name,
                             mesh = mesh,
                             dclass = self._dclass_vis)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.summit_body.add('geom',
                             name = prefix+'\\viz\\'+wheel_name+'_cylinder',
                             dclass = self._dclass_col,
                             size = self._size,
                             type = 'cylinder',
                             friction = self._friction)
        # Add Inertial properties
        self.summit_body.add('inertial',
                             pos = (0, 0, 0), 
                             mass = self._mass,
                             diaginertia = self._inertia)
        # Add Joint
        self.summit_body.add('joint',
                             name = prefix+'\\'+wheel_name.replace("\\","_"),
                             dclass = self._dclass_joint,
                             pos = (0,0,0))
        
# Make a class for constructing the Summit body
class Summit:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _body_mesh = 'base_link_summit'
    _camera_fovy = [55,
                    55,
                    90]
    _camera_name = ['\\front\\camera\\intel',
                    '\\rear\\camera\\intel',
                    '\\pole_link\\camera\\intel']
    _camera_pos = [[0, 0.362, 0.373],
                   [0,-0.362- 0.373]]
    _camera_quat = [[0, 0, 0.707107, 0.707107],
                    [0.707107, 0.707107, 0, 0],
                    [0.707107, 0.707107, 0, 0]]
    _com_pos = (0, 0, 0.37)
    _inertia = (1.391, 6.853, 6.125)
    _mass = 125
    _name_summit = 'smt'
    _name_base_link = '\\base_link'
    _rgba = (0.8, 0.35, 0.1, 1)
    _site_name = 'site'
    _wam_name = 'wam_7dof_bhand'
    _wam_position = (0, 0.14, 0.405)
    _wam_quat = (0.707, 0, 0, 0.707)
    _wheel_mesh = ['mecanum_LF_1',
                   'mecanum_LR_1',
                   'mecanum_RF_1',
                   'mecanum_RR_1']
    _wheel_name = ['whl\\LF',
                   'whl\\LR',
                   'whl\\RF',
                   'whl\\RR']
    _wheel_pos = [[-0.220, 0.222, 0.128],
                  [-0.220,-0.223, 0.128],
                  [ 0.220, 0.222, 0.128],
                  [ 0.220,-0.223, 0.128]]
    _wheel_quat = (0.707107, 0, 0.707106, 0)
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, prefix=''):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=self._name_summit+'_'+prefix+self._name_base_link)
        # Add some classes
        summit_class = self.mjcf_model.default.add('default', dclass = 'summit\\body\\viz')
        # summit_class.site.set_attributes('geom' ,rgba = (0.95, 0.6, 0.05, 1))
        # Add a body element:
        self.summit_body = self.mjcf_model.worldbody.add('body',
                                                         name = self._name_summit+'_'+prefix,
                                                         pos = (0, 0, 0))
        # Add Inertial properties
        self.summit_body.add('inertial',
                             pos = self._com_pos, 
                             mass = self._mass,
                             diaginertia = self._inertia)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.summit_body.add('geom',
                             name = self._name_summit+'_'+prefix+'\\viz\\'+self._name_base_link,
                             mesh = self._body_mesh,
                             dclass = 'summit\\body\\viz')
        
        # Add camera locations
        # An empty array to hold box-sites
        camera_site = []
        for x in range(len(self._camera_pos)):
            camera_site.append(self.summit_body.add('site',
                                                    pos = self._camera_pos[x],
                                                    quat = self._camera_quat[x],
                                                    dclass = self._site_name))
        # Create camera
        # Initialize empty list of camera
        cameras = []
        for x in range(len(self._camera_pos)):
            cameras.append(Camera(name = self._name_summit+'_'+prefix+self._camera_name[x],
                                  fovy = self._camera_fovy[x]))
        # Attach the boxes
        for x in range(len(self._camera_pos)):
            camera_site[x].attach(cameras[x].mjcf_model)
        
        # Add a cylindrical-pole
        self.pole = self.summit_body.add('body',
                                         name = self._name_summit+'_'+prefix+'\\pole_link',
                                         pos = (0, 0.32, 1))
        self.pole.add('geom',
                      name = self._name_summit+'_'+prefix+'\\viz\\pole_link',
                      dclass = 'summit\\wheel\\viz',
                      type = 'cylinder',
                      size = (0.005, 0.42))
        # Add a camera to the pole
        pole_camera_site = self.pole.add('site',
                                         pos = (0, 0, 0.44),
                                         quat = self._camera_quat[2])
        pole_camera = Camera(name = self._name_summit+'_'+prefix+self._camera_name[2],
                             fovy = self._camera_fovy[2])
        pole_camera_site.attach(pole_camera.mjcf_model)

        # Add collision bodies
        self.summit_body.add('geom',
                             name = self._name_summit+'_'+prefix+'\\col\\'+self._name_base_link+'\\box_body',
                             dclass = 'summit\\col',
                             pos = (0, 0, 0.42),
                             size = (0.24, 0.32, 0.16),
                             type = 'box')
        self.summit_body.add('geom',
                             name = self._name_summit+'_'+prefix+'\\col\\'+self._name_base_link+'\\arm_rest',
                             dclass = 'summit\\col',
                             pos = (0, 0.098, 0.55),
                             quat = (0.5, 0.5, -0.5, -0.5),
                             mesh = 'arm_rest')
        self.summit_body.add('geom',
                             name = self._name_summit+'_'+prefix+'\\col\\'+self._name_base_link+'\\cylinder_lidar',
                             dclass = 'summit\\col',
                             pos = (0.246, 0.362, 0.313),
                             size = (0.06, 0.044),
                             type = 'cylinder')
        
        # Add joints
        # World placement
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\world_x',
                             armature = 0.0001,
                             axis = (0, 1, 0),
                             damping = 1e+11,
                             pos = (0, 0, 0),
                             type = 'slide')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\world_y',
                             armature = 0.0001,
                             axis = (-1, 0, 0),
                             damping = 1e+11,
                             pos = (0, 0, 0),
                             type = 'slide')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\world_z',
                             armature = 0.0001,
                             axis = (0, 0, 1),
                             damping = 1e+0,
                             pos = (0, 0, 0),
                             type = 'slide')
        # Control summit base
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\pose\\x',
                             axis = (0, 1, 0),
                             damping = 15,
                             pos = (0, 0, 0.4),
                             type = 'slide')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\pose\\y',
                             axis = (-1, 0, 0),
                             damping = 15,
                             pos = (0, 0, 0.4),
                             type = 'slide')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\orie\\z',
                             axis = (0, 0, 1),
                             damping = 10,
                             pos = (0, 0, 0.4),
                             type = 'hinge')
        
        # Add wheels
        # An empty array to hold box-sites
        wheel_site = []
        for x in range(len(self._camera_pos)):
            wheel_site.append(self.summit_body.add('site',
                                                   pos = self._wheel_pos[x],
                                                   quat = self._wheel_quat,
                                                   dclass = self._site_name))
        # Create wheels
        # Initialize empty list of wheels
        wheels = []
        for x in range(len(self._camera_pos)):
            wheels.append(Wheel(prefix = self._name_summit+'_'+prefix, wheel_name = self._wheel_name[x],
                                mesh = self._wheel_mesh[x]))
        # Attach the boxes
        for x in range(len(self._camera_pos)):
            wheel_site[x].attach(wheels[x].mjcf_model)

        # Add WAM
        wam_site = self.pole.add('site',
                                 pos = self._wam_position,
                                 quat = self._wam_quat)
        wam = WAM(name = self._wam_name+'_'+prefix)
        wam_site.attach(wam.mjcf_model)



def export_with_assets(mjcf_model, out_dir, out_file_name=None,
                       *,
                       precision=mjcf.constants.XML_DEFAULT_PRECISION, # List of keywords : (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/mjcf/constants.py#L30)
                       zero_threshold=0):
  """Saves mjcf.model in the given directory in MJCF (XML) format.

  Creates an MJCF XML file named `out_file_name` in the specified `out_dir`, and
  writes all of its assets into the same directory.

  Args:
    mjcf_model: `mjcf.RootElement` instance to export.
    out_dir: Directory to save the model and assets. Will be created if it does
      not already exist.
    out_file_name: (Optional) Name of the XML file to create. Defaults to the
      model name (`mjcf_model.model`) suffixed with '.xml'.
    precision: (optional) Number of digits to output for floating point
      quantities.
    zero_threshold: (optional) When outputting XML, floating point quantities
      whose absolute value falls below this threshold will be treated as zero.

  Raises:
    ValueError: If `out_file_name` is a string that does not end with '.xml'.
  """
  if out_file_name is None:
    out_file_name = mjcf_model.model + '.xml'
  elif not out_file_name.lower().endswith('.xml'):
    raise ValueError('If `out_file_name` is specified it must end with '
                     '\'.xml\': got {}'.format(out_file_name))
  assets = mjcf_model.get_assets()
  # This should never happen because `mjcf` does not support `.xml` assets.
  assert out_file_name not in assets
  assets[out_file_name] = mjcf_model.to_xml_string(
      precision=precision, zero_threshold=zero_threshold)
  if not os.path.exists(out_dir):
    os.makedirs(out_dir)
  for filename, contents in assets.items():
    with open(os.path.join(out_dir, filename), 'wb') as f:
      f.write(util.to_binary_string(contents))

# Create a Summit
summit_body = Summit()
folder_name = '/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/components'
export_with_assets(mjcf_model = summit_body.mjcf_model, out_dir=folder_name)

# # Read the created file
# filename = folder_name+'/include_summit_wam_bhand_Chain.xml'
# mjcf_model = mjcf.from_path(filename, escape_separators=True)

# geoms = mjcf_model.find_all('geom')
# print(geoms[0].name)
# # print(geoms[0].dclass)
