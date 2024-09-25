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
    _name = '\\intel\\rgb'
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
                             fovy = fovy,
                             dclass = 'rand')

# Make a class for constructing a link with joints
class Link_w_Joints:
    #===================#
    #  C O N S T A N T  #
    #===================#
    
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    # joint axis default value [0, 0, 1]: https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint
    def __init__(self, col_meshes, com_pos, dclass_col, inertia, joint_damping, joint_frictionloss, joint_name, joint_range, mass, name, viz_meshes, dclass_viz = None, joint_axis = [0, 0, 1]):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.link_body = self.mjcf_model.worldbody.add('body',
                                                       name = name)
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
                           axis = joint_axis,
                           frictionloss = joint_frictionloss,
                           dclass = 'rand')
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        # For visualization
        self.link_body.add('geom',
                           dclass = name+'\\viz' if dclass_viz is None else dclass_viz,
                           mesh = viz_meshes)
        # For Collision
        for x in range(len(col_meshes)):
            self.link_body.add('geom',
                               dclass = dclass_col,
                               mesh = col_meshes[x])

# Make a class for constructing the 'forearm_link' with joints
class Forearm_link:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _col_mesh_euler = [[0, 0, 0],
                       [1.57, 0, 0],
                       [1.57, 0, 0]]
    _col_mesh_pos = [[0, 0, 0],
                     [-0.045, -0.073, 0],
                     [-0.045, -0.073, 0]]
    _viz_mesh_euler = [[0, 0, 0],
                       [1.57, 0, 0]]
    _viz_mesh_pos = [[0, 0, 0],
                     [-0.045, -0.073, 0]]
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, col_meshes, com_pos, dclass_col, inertia, joint_damping, joint_frictionloss, joint_name, joint_range, mass, name, viz_dclass_name, viz_meshes):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.link_body = self.mjcf_model.worldbody.add('body',
                                                       name = name)
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
                           frictionloss = joint_frictionloss,
                           dclass = 'rand')
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        # For visualization
        for x in range(len(viz_meshes)):
            self.link_body.add('geom',
                              dclass = viz_dclass_name[x]+'\\viz',
                              euler = self._viz_mesh_euler[x],
                              mesh = viz_meshes[x],
                              pos = self._viz_mesh_pos[x])
        # For Collision
        for x in range(len(col_meshes)):
            self.link_body.add('geom',
                               dclass = dclass_col,
                               euler = self._col_mesh_euler[x],
                               mesh = col_meshes[x],
                              pos = self._col_mesh_pos[x])

# Make a class for constructing a link without joints
class Link_wo_Joints:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _dclass_col = 'wam\\col'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, com_pos, dclass_col, inertia, mass, name, viz_meshes, col_meshes = None):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element:
        self.link_body = self.mjcf_model.worldbody.add('body',
                                                       name = name)
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
        if col_meshes is not None:
          for x in range(len(col_meshes)):
              self.link_body.add('geom',
                                dclass = dclass_col,
                                mesh = col_meshes[x])

# Make a class for constructing the BHand
class Bhand:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _bhand_name = 'bhand'
    _bhand_model_name = 'BH8-280'
    _body_name = ['palm',
                  'prox',
                  'med',
                  'dist']
    _col_mesh_name = [['bhand_palm_link_convex_decomposition_p1',
                       'bhand_palm_link_convex_decomposition_p2',
                       'bhand_palm_link_convex_decomposition_p3',
                       'bhand_palm_link_convex_decomposition_p4'],
                      ['bhand_finger_prox_link_convex_decomposition_p1',
                       'bhand_finger_prox_link_convex_decomposition_p2',
                       'bhand_finger_prox_link_convex_decomposition_p3'],
                      ['bhand_finger_med_link_convex'],
                      ['bhand_finger_dist_link_convex']]
    _com_pos = [[5.0019e-005,  0.0044561, 0.037268],
                [0.030616, 7.3219e-005, 0.011201],
                [0.023133, 0.00078642, 0.00052792],
                [0.02295, 0.0010739, 0.00041752]]
    _euler_dist = [0, 0, -0.84]
    _finger_name = ['finger_1',
                    'finger_2',
                    'finger_3']
    _finger_name_short = ['f1',
                     'f2',
                     'f3']
    _inertial = [[0.0006986, 0.00050354, 0.00062253, 2.7577e-007, -7.8138e-007, -6.44e-005],
                 [2.0672e-005, 7.4105e-005, 6.8207e-005, 2.6024e-007, 6.3481e-006, 1.7118e-008],
                 [4.8162e-006, 4.3317e-005, 4.4441e-005, 5.7981e-007, -7.2483e-007, -2.6653e-009],
                 [3.1199e-006, 1.6948e-005, 1.5809e-005, 4.5115e-007, -2.9813e-007, -1.8635e-008]]
    _joint_axis = [[0, 0, -1],
                   [0, 0, 1],
                   [0, 0, 1]]
    _joint_damping = 0.1
    _joint_frictionloss = 0.1
    _joint_range = [[-0.1, 3.2],
                    [-0.1, 2.5],
                    [-0.1, 0.9]]
    _mass = [0.60858,
             0.14109,
             0.062139,
             0.04166]
    _pos = [[0, 0, 0],
            [-0.025, 0, 0.0415],
            [0.025, 0, 0.0415],
            [0, 0.05, 0.0754],
            [0.05, 0, 0.0339],
            [0.06994, 0.003, 0]]
    _quat = [[1, 0, 0, 0], # quant=w x y z
             [0.707107, 0, 0, -0.707107],
             [0.707107, 0, 0, -0.707107],
             [0.5, 0.5, 0.5, 0.5],
             [0.707107, 0.707107, 0, 0],
             [0.92388, 0, 0, 0.382683]]
    _site_name = 'site'
    _viz_mesh_name = ['bhand_palm_fine',
                      'bhand_finger_prox_link_fine',
                      'bhand_finger_med_link_fine',
                      'bhand_finger_dist_link_fine']
    _wam_name = 'wam'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name, prefix):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add 'base_link' element:
        self.base_link = self.mjcf_model.worldbody.add('body',
                                                       name = self._bhand_model_name)
        
        # Attach site for 'palm link'
        self.palm_link_site = self.base_link.add('site',
                                                  pos = self._pos[0],
                                                  quat = self._quat[0],
                                                  dclass = self._site_name)
        # Create 'palm link' 
        self.palm_link = Link_wo_Joints(col_meshes = self._col_mesh_name[0],
                                        com_pos = self._com_pos[0],
                                        dclass_col = 'wam\\bhand\\col',
                                        inertia = self._inertial[0],
                                        mass = self._mass[0],
                                        name = self._wam_name+'\\'+self._bhand_name+'\\'+self._bhand_name+'_'+self._body_name[0]+'_link',
                                        viz_meshes = self._viz_mesh_name[0])
        # Attach 'palm link'
        self.palm_link_site.attach(self.palm_link.mjcf_model)

        # Empty arrays
        self.finger_index_prox_link_site = []
        self.finger_index_prox_link = []
        self.finger_index_med_link_site = []
        self.finger_index_med_link = []
        self.finger_index_dist_link_site = []
        self.finger_index_dist_link = []
        for finger_index in range(3):
          # Finger i
          # Attach site for 'finger 1 prox link'
          if finger_index in range(2):
            self.finger_index_prox_link_site.append(self.palm_link.mjcf_model.worldbody.add('site',
                                                                                  pos = self._pos[1+finger_index],
                                                                                  quat = self._quat[1+finger_index],
                                                                                  dclass = self._site_name))
            # Create 'finger 1 prox link' 
            self.finger_index_prox_link.append(Link_w_Joints(col_meshes = self._col_mesh_name[1],
                                                             com_pos = self._com_pos[1],
                                                             dclass_col = self._wam_name+'\\'+self._bhand_name+'\\col',
                                                             dclass_viz = self._wam_name+'\\'+self._bhand_name+'\\'+self._bhand_name+'_finger_'+self._body_name[1]+'_link\\viz',
                                                             inertia = self._inertial[1],
                                                             joint_axis = self._joint_axis[0],
                                                             joint_damping = self._joint_damping,
                                                             joint_frictionloss = self._joint_frictionloss,
                                                             joint_name = self._bhand_name+'\\'+self._finger_name_short[0+finger_index]+'\\'+self._body_name[1],
                                                             joint_range = self._joint_range[0],
                                                             mass = self._mass[1],
                                                             name = self._wam_name+'\\'+self._bhand_name+'\\'+self._finger_name[0+finger_index]+'\\'+self._body_name[1]+'_link',
                                                             viz_meshes = self._viz_mesh_name[1]))
            # Attach 'finger 1 prox link'
            self.finger_index_prox_link_site[finger_index].attach(self.finger_index_prox_link[finger_index].mjcf_model)

            # Attach site for 'finger 1 med link'
            self.finger_index_med_link_site.append(self.finger_index_prox_link[finger_index].mjcf_model.worldbody.add('site',
                                                                                                            pos = self._pos[4],
                                                                                                            quat = self._quat[4],
                                                                                                            dclass = self._site_name))
          else:
             # Attach site for 'finger 1 med link' at 'palm link'
            self.finger_index_med_link_site.append(self.palm_link.mjcf_model.worldbody.add('site',
                                                                                 pos = self._pos[1+finger_index],
                                                                                 quat = self._quat[1+finger_index],
                                                                                 dclass = self._site_name))
          # Create 'finger 1 med link' 
          self.finger_index_med_link.append(Link_w_Joints(col_meshes = self._col_mesh_name[2],
                                                          com_pos = self._com_pos[2],
                                                          dclass_col = self._wam_name+'\\'+self._bhand_name+'\\col',
                                                          dclass_viz = self._wam_name+'\\'+self._bhand_name+'\\'+self._bhand_name+'_finger_'+self._body_name[2]+'_link\\viz',
                                                          inertia = self._inertial[2],
                                                          joint_axis = self._joint_axis[1],
                                                          joint_damping = self._joint_damping,
                                                          joint_frictionloss = self._joint_frictionloss,
                                                          joint_name = self._bhand_name+'\\'+self._finger_name_short[0+finger_index]+'\\'+self._body_name[2],
                                                          joint_range = self._joint_range[1],
                                                          mass = self._mass[2],
                                                          name = self._wam_name+'\\'+self._bhand_name+'\\'+self._finger_name[0+finger_index]+'\\'+self._body_name[2]+'_link',
                                                          viz_meshes = self._viz_mesh_name[2]))
          # Attach 'finger 1 med link'
          self.finger_index_med_link_site[finger_index].attach(self.finger_index_med_link[finger_index].mjcf_model)
          
          # Attach site for 'finger 1 dist link'
          self.finger_index_dist_link_site.append(self.finger_index_med_link[finger_index].mjcf_model.worldbody.add('site',
                                                                                                          pos = self._pos[5],
                                                                                                          quat = self._quat[5],
                                                                                                          dclass = self._site_name))
          # Create 'finger 1 dist link' 
          self.finger_index_dist_link.append(Link_w_Joints(col_meshes = self._col_mesh_name[3],
                                                           com_pos = self._com_pos[3],
                                                           dclass_col = self._wam_name+'\\'+self._bhand_name+'\\col',
                                                           dclass_viz = self._wam_name+'\\'+self._bhand_name+'\\'+self._bhand_name+'_finger_'+self._body_name[3]+'_link\\viz',
                                                           inertia = self._inertial[3],
                                                           joint_axis = self._joint_axis[2],
                                                           joint_damping = self._joint_damping,
                                                           joint_frictionloss = self._joint_frictionloss,
                                                           joint_name = self._bhand_name+'\\'+self._finger_name_short[0+finger_index]+'\\'+self._body_name[3],
                                                           joint_range = self._joint_range[2],
                                                           mass = self._mass[3],
                                                           name = self._wam_name+'\\'+self._bhand_name+'\\'+self._finger_name[0+finger_index]+'\\'+self._body_name[3]+'_link',
                                                           viz_meshes = self._viz_mesh_name[3]))
          # Attach 'finger 1 dist link'
          self.finger_index_dist_link_site[finger_index].attach(self.finger_index_dist_link[finger_index].mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.base_link.site.clear()
        self.palm_link.mjcf_model.worldbody.site.clear()
        for finger_prox_link in self.finger_index_prox_link:
           finger_prox_link.mjcf_model.worldbody.site.clear()
        for finger_med_link in self.finger_index_med_link:
           finger_med_link.mjcf_model.worldbody.site.clear()

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
    _bhand_name = 'bhand'
    _dclass_col = 'wam\\col'
    _camera_pos = [0, 0.085, 0.02]
    _camera_name = "camera"
    _camera_fovy = 55
    _col_mesh_names = [['base_link_convex'],
                       ['shoulder_link_convex_decomposition_p1',
                        'shoulder_link_convex_decomposition_p2',
                        'shoulder_link_convex_decomposition_p3'],
                       ['shoulder_pitch_link_convex'],
                       ['upper_arm_link_convex_decomposition_p1',
                        'upper_arm_link_convex_decomposition_p2'],
                       ['elbow_link_convex',
                        'forearm_link_convex_decomposition_p1',
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
    _joint_name = ['\\J1',
                   '\\J2',
                   '\\J3',
                   '\\J4',
                   '\\J5',
                   '\\J6',
                   '\\J7']
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
            [0, 0, 0.06],
            [0, 0.01, 0]]
    # quant=w x y z 
    _quat = [[1, 0, 0, 0],
             [1, 0, 0, 0],
             [0.707107, -0.707107, 0, 0],
             [0.707107,  0.707107, 0, 0],
             [0.707107, -0.707107, 0, 0],
             [0.707107, 0.707107, 0, 0],
             [0.707107, -0.707107, 0, 0],
             [0.707107, 0.707107, 0, 0],
             [0.707107, 0.707107, 0, 0],
             [0, 0, 0.707107, 0.707107]]
    _site_name = 'site'
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
    _wam_name = 'wam'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name, prefix):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name+'_'+prefix)
        # Add 'base_link' element:
        self.base_link = self.mjcf_model.worldbody.add('body',
                                                       name = self._wam_name+'_'+prefix+'\\'+self._body_name[0],
                                                       pos = (0, 0, 0))
        
        # Attach site for 'base'
        self.base_site = self.base_link.add('site',
                                            pos = self._pos[0],
                                            quat = self._quat[0],
                                            dclass = self._site_name)
        # Create link 'base'
        self.base = Link_wo_Joints(col_meshes = self._col_mesh_names[0],
                                   com_pos = self._com_pos[0],
                                   dclass_col = 'wam\\col',
                                   inertia = self._inertial[0],
                                   mass = self._mass[0],
                                   name = self._wam_name+'_'+prefix+'\\'+self._body_name[1],
                                   viz_meshes = self._viz_mesh_names[0])
        # Attach 'base'
        self.base_site.attach(self.base.mjcf_model)

        # Attach site for 'shoulder_yaw_link'
        self.shoulder_yaw_link_site = self.base_link.add('site',
                                                         pos = self._pos[1],
                                                         quat = self._quat[1],
                                                         dclass = self._site_name)
        # Create link 'shoulder_yaw_link'
        self.shoulder_yaw_link = Link_w_Joints(col_meshes = self._col_mesh_names[1],
                                               com_pos = self._com_pos[1],
                                               dclass_col = 'wam\\col',
                                               inertia = self._inertial[1], 
                                               joint_damping = self._joint_damping[0],
                                               joint_frictionloss = self._joint_frictionloss,
                                               joint_name = self._wam_name+'_'+prefix+self._joint_name[0],
                                               joint_range = self._joint_range[0],
                                               mass = self._mass[1],
                                               name = self._wam_name+'_'+prefix+'\\'+self._body_name[2],
                                               viz_meshes = self._viz_mesh_names[1])
        # Attach 'shoulder_yaw_link'
        self.shoulder_yaw_link_site.attach(self.shoulder_yaw_link.mjcf_model)
        
        # Attach site for 'shoulder_pitch_link'
        self.shoulder_pitch_link_site = self.shoulder_yaw_link.mjcf_model.worldbody.add('site',
                                                                                        pos = self._pos[2],
                                                                                        quat = self._quat[2],
                                                                                        dclass = self._site_name)
        # Create link 'shoulder_pitch_link'
        self.shoulder_pitch_link = Link_w_Joints(col_meshes = self._col_mesh_names[2],
                                                 com_pos = self._com_pos[2],
                                                 dclass_col = 'wam\\col',
                                                 inertia = self._inertial[2], 
                                                 joint_damping = self._joint_damping[1],
                                                 joint_frictionloss = self._joint_frictionloss,
                                                 joint_name = self._wam_name+'_'+prefix+self._joint_name[1],
                                                 joint_range = self._joint_range[1],
                                                 mass = self._mass[2],
                                                 name = self._wam_name+'_'+prefix+'\\'+self._body_name[3],
                                                 viz_meshes = self._viz_mesh_names[2])
        # Attach 'shoulder_pitch_link'
        self.shoulder_pitch_link_site.attach(self.shoulder_pitch_link.mjcf_model)
        
        # Attach site for 'upper_arm_link'
        self.upper_arm_link_site = self.shoulder_pitch_link.mjcf_model.worldbody.add('site',
                                                                                     pos = self._pos[3],
                                                                                     quat = self._quat[3],
                                                                                     dclass = self._site_name)
        # Create link 'upper_arm_link'
        self.upper_arm_link = Link_w_Joints(col_meshes = self._col_mesh_names[3],
                                            com_pos = self._com_pos[3],
                                            dclass_col = 'wam\\col',
                                            inertia = self._inertial[3], 
                                            joint_damping = self._joint_damping[2],
                                            joint_frictionloss = self._joint_frictionloss,
                                            joint_name = self._wam_name+'_'+prefix+self._joint_name[2],
                                            joint_range = self._joint_range[2],
                                            mass = self._mass[3],
                                            name = self._wam_name+'_'+prefix+'\\'+self._body_name[4],
                                            viz_meshes = self._viz_mesh_names[3])
        # Attach 'upper_arm_link'
        self.upper_arm_link_site.attach(self.upper_arm_link.mjcf_model)

        # Attach site for 'forearm_link'
        self.forearm_link_site = self.upper_arm_link.mjcf_model.worldbody.add('site',
                                                                              pos = self._pos[4],
                                                                              quat = self._quat[4],
                                                                              dclass = self._site_name)
        # Create link 'forearm_link'
        self.forearm_link = Forearm_link(col_meshes = self._col_mesh_names[4],
                                         com_pos = self._com_pos[4],
                                         dclass_col = 'wam\\col',
                                         inertia = self._inertial[4], 
                                         joint_damping = self._joint_damping[3],
                                         joint_frictionloss = self._joint_frictionloss,
                                         joint_name = self._wam_name+'_'+prefix+self._joint_name[3],
                                         joint_range = self._joint_range[3],
                                         mass = self._mass[4],
                                         name = self._wam_name+'_'+prefix+'\\'+self._body_name[5],
                                         viz_dclass_name = [self._wam_name+'_'+prefix+'\\elbow_link',
                                                            self._wam_name+'_'+prefix+'\\fore_arm_link'],
                                         viz_meshes = self._viz_mesh_names[4:6])
        # Attach 'forearm_link'
        self.forearm_link_site.attach(self.forearm_link.mjcf_model)

        # Attach site for 'wrist_yaw_link'
        self.wrist_yaw_link_site = self.forearm_link.mjcf_model.worldbody.add('site',
                                                                              pos = self._pos[5],
                                                                              quat = self._quat[5],
                                                                              dclass = self._site_name)
        # Create link 'wrist_yaw_link'
        self.wrist_yaw_link = Link_w_Joints(col_meshes = self._col_mesh_names[5],
                                            com_pos = self._com_pos[5],
                                            dclass_col = 'wam\\col',
                                            inertia = self._inertial[5], 
                                            joint_damping = self._joint_damping[4],
                                            joint_frictionloss = self._joint_frictionloss,
                                            joint_name = self._wam_name+'_'+prefix+self._joint_name[4],
                                            joint_range = self._joint_range[4],
                                            mass = self._mass[5],
                                            name = self._wam_name+'_'+prefix+'\\'+self._body_name[6],
                                            viz_meshes = self._viz_mesh_names[6])
        # Attach 'wrist_yaw_link'
        self.wrist_yaw_link_site.attach(self.wrist_yaw_link.mjcf_model)

        # Attach site for 'wrist_pitch_link'
        self.wrist_pitch_link_site = self.wrist_yaw_link.mjcf_model.worldbody.add('site',
                                                                                  pos = self._pos[6],
                                                                                  quat = self._quat[6],
                                                                                  dclass = self._site_name)
        # Create link 'wrist_pitch_link'
        self.wrist_pitch_link = Link_w_Joints(col_meshes = self._col_mesh_names[6],
                                              com_pos = self._com_pos[6],
                                              dclass_col = 'wam\\col',
                                              inertia = self._inertial[6], 
                                              joint_damping = self._joint_damping[5],
                                              joint_frictionloss = self._joint_frictionloss,
                                              joint_name = self._wam_name+'_'+prefix+self._joint_name[5],
                                              joint_range = self._joint_range[5],
                                              mass = self._mass[6],
                                              name = self._wam_name+'_'+prefix+'\\'+self._body_name[7],
                                              viz_meshes = self._viz_mesh_names[7])
        # Attach 'wrist_pitch_link'
        self.wrist_pitch_link_site.attach(self.wrist_pitch_link.mjcf_model)

        # Attach site for 'wrist_palm_link'
        self.wrist_palm_link_site = self.wrist_pitch_link.mjcf_model.worldbody.add('site',
                                                                                   pos = self._pos[7],
                                                                                   quat = self._quat[7],
                                                                                   dclass = self._site_name)
        # Create link 'wrist_palm_link'
        self.wrist_palm_link = Link_w_Joints(col_meshes = self._col_mesh_names[7],
                                             com_pos = self._com_pos[7],
                                             dclass_col = 'wam\\col',
                                             inertia = self._inertial[7], 
                                             joint_damping = self._joint_damping[6],
                                             joint_frictionloss = self._joint_frictionloss,
                                             joint_name = self._wam_name+'_'+prefix+self._joint_name[6],
                                             joint_range = self._joint_range[6],
                                             mass = self._mass[7],
                                             name = self._wam_name+'_'+prefix+'\\'+self._body_name[8],
                                             viz_meshes = self._viz_mesh_names[8])
        # Attach 'wrist_palm_link'
        self.wrist_palm_link_site.attach(self.wrist_palm_link.mjcf_model)

        # Attach site for 'camera'
        self.wrist_camera_site = self.wrist_palm_link.mjcf_model.worldbody.add('site',
                                                                               pos = self._camera_pos,
                                                                               dclass = self._site_name)
        # Create camera 'camera'
        self.wrist_camera = Camera(name = self._wam_name+'_'+prefix+'\\'+self._camera_name,
                                   fovy = self._camera_fovy)
        # Attach 'camera'
        self.wrist_camera_site.attach(self.wrist_camera.mjcf_model)

        # Attach site for 'torque_sensor_link'
        self.wrist_torque_link_site = self.wrist_palm_link.mjcf_model.worldbody.add('site',
                                                                                    pos = self._pos[8],
                                                                                    quat = self._quat[8],
                                                                                    dclass = self._site_name)
        # Create link 'torque_sensor_link'
        self.wrist_torque_link = Link_wo_Joints(com_pos = self._com_pos[8],
                                                dclass_col = 'wam\\col',
                                                inertia = self._inertial[8],
                                                mass = self._mass[8],
                                                name = self._wam_name+'_'+prefix+'\\'+self._body_name[9],
                                                viz_meshes = self._viz_mesh_names[9])
        # Attach 'torque_sensor_link'
        self.wrist_torque_link_site.attach(self.wrist_torque_link.mjcf_model)

        # Attach site for 'bhand'
        self.bhand_site = self.wrist_torque_link.mjcf_model.worldbody.add('site',
                                                                          pos = self._pos[9],
                                                                          quat = self._quat[9],
                                                                          dclass = self._site_name)
        # Create link 'torque_sensor_link'
        self.bhand_link = Bhand(name = self._wam_name+'_'+prefix+'\\'+self._bhand_name,
                                prefix = prefix)
        # Attach 'torque_sensor_link'
        self.bhand_site.attach(self.bhand_link.mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.base_link.site.clear()
        self.shoulder_yaw_link.mjcf_model.worldbody.site.clear()
        self.shoulder_pitch_link.mjcf_model.worldbody.site.clear()
        self.upper_arm_link.mjcf_model.worldbody.site.clear()
        self.forearm_link.mjcf_model.worldbody.site.clear()
        self.wrist_yaw_link.mjcf_model.worldbody.site.clear()
        self.wrist_pitch_link.mjcf_model.worldbody.site.clear()
        self.wrist_palm_link.mjcf_model.worldbody.site.clear()
        self.wrist_torque_link.mjcf_model.worldbody.site.clear()

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
    def __init__(self, mesh, orientation, position, prefix, wheel_name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=prefix+'\\'+wheel_name+'_link')
        # Add a body element:
        self.summit_body = self.mjcf_model.worldbody.add('body',
                                                         name = prefix+'\\'+wheel_name+'_link')
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.summit_body.add('geom',
                             name = prefix+'\\viz\\'+wheel_name,
                             mesh = mesh,
                             dclass = self._dclass_vis)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.summit_body.add('geom',
                             name = prefix+'\\viz\\'+wheel_name+'_cylinder',
                             dclass = self._dclass_col,
                             pos = position,
                             quat = orientation,
                             size = self._size,
                             type = 'cylinder',
                             friction = self._friction)
        # Add Inertial properties
        self.summit_body.add('inertial',
                             pos = position, 
                             mass = self._mass,
                             diaginertia = self._inertia)
        # Add Joint
        self.summit_body.add('joint',
                             name = prefix+'\\'+wheel_name.replace("\\","_"),
                             dclass = self._dclass_joint,
                             pos = position)
        
# Make a class for constructing the Summit body
class Summit:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _body_mesh = 'base_link_summit'
    _camera_fovy = [55,
                    55,
                    90]
    _camera_name = ['\\front\\camera',
                    '\\rear\\camera',
                    '\\pole_link\\camera']
    _camera_pos = [[0, 0.362, 0.373],
                   [0,-0.362, 0.373]]
    _camera_quat = [[0, 0, 0.707107, 0.707107],
                    [0.707107, 0.707107, 0, 0],
                    [0.707107, 0.707107, 0, 0]]
    _com_pos = (0, 0, 0.37)
    _inertia = (1.391, 6.853, 6.125)
    _mass = 125
    _name_summit = 'smt'
    _name_base_link = 'base_link'
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
        self.mjcf_model = mjcf.RootElement(model=self._name_summit+'_'+prefix+'\\'+self._name_base_link)
        # # Add some classes
        # summit_class = self.mjcf_model.default.add('default', dclass = 'summit\\body\\viz')
        # summit_class.site.set_attributes('geom' ,rgba = (0.95, 0.6, 0.05, 1))
        # Add a body element:
        self.summit_body = self.mjcf_model.worldbody.add('body',
                                                         name = self._name_summit+'_'+prefix+'\\'+self._name_base_link,
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
        # An empty array to hold camera-sites
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
        # Attach the cameras
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
                             type = 'slide',
                             dclass = 'rand')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\world_y',
                             armature = 0.0001,
                             axis = (-1, 0, 0),
                             damping = 1e+11,
                             pos = (0, 0, 0),
                             type = 'slide',
                             dclass = 'rand')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\world_z',
                             armature = 0.0001,
                             axis = (0, 0, 1),
                             damping = 1e+0,
                             pos = (0, 0, 0),
                             type = 'slide',
                             dclass = 'rand')
        # Control summit base
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\pose\\x',
                             axis = (0, 1, 0),
                             damping = 15,
                             pos = (0, 0, 0.4),
                             type = 'slide',
                             dclass = 'rand')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\pose\\y',
                             axis = (-1, 0, 0),
                             damping = 15,
                             pos = (0, 0, 0.4),
                             type = 'slide',
                             dclass = 'rand')
        self.summit_body.add('joint',
                             name = self._name_summit+'_'+prefix+'\\orie\\z',
                             axis = (0, 0, 1),
                             damping = 10,
                             pos = (0, 0, 0.4),
                             type = 'hinge',
                             dclass = 'rand')
        
        # Add wheels
        # An empty array to hold box-sites
        wheel_site = []
        for x in range(len(self._wheel_pos)):
            wheel_site.append(self.summit_body.add('site',
                                                   pos = (0, 0, 0),
                                                   dclass = self._site_name))
        # Create wheels
        # Initialize empty list of wheels
        wheels = []
        for x in range(len(self._wheel_pos)):
            wheels.append(Wheel(orientation = self._wheel_quat,
                                position = self._wheel_pos[x],
                                prefix = self._name_summit+'_'+prefix,
                                wheel_name = self._wheel_name[x],
                                mesh = self._wheel_mesh[x]))
        # Attach the boxes
        for x in range(len(self._wheel_pos)):
            wheel_site[x].attach(wheels[x].mjcf_model)

        # Add WAM
        wam_site = self.summit_body.add('site',
                                        pos = self._wam_position,
                                        quat = self._wam_quat)
        wam = WAM(name = self._wam_name,
                  prefix = prefix)
        wam_site.attach(wam.mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.summit_body.site.clear()
        self.pole.site.clear()


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
prefix = '1'
summit_body = Summit(prefix = prefix)
folder_name = '/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/components'
export_with_assets(mjcf_model = summit_body.mjcf_model, out_dir=folder_name)

# Read the created file
filename = folder_name+'/smt_'+prefix+'\\base_link.xml'
mjcf_model = mjcf.from_path(filename, escape_separators=True)

bodies = mjcf_model.find_all('body')
cameras = mjcf_model.find_all('camera')
geoms = mjcf_model.find_all('geom')
joints = mjcf_model.find_all('joint')

camera_names =['smt_'+prefix+'\\front\\camera\\intel\\rgb',
               'smt_'+prefix+'\\rear\\camera\\intel\\rgb',
               'smt_'+prefix+'\\pole_link\\camera\\intel\\rgb',
               'wam\\camera\\intel\\rgb']

class_names_bhand = ['bhand_palm_link',
                     'bhand_finger_prox_link',
                     'bhand_finger_med_link',
                     'bhand_finger_dist_link']

class_names_wam = ['base',
                   'shoulder_yaw_link',
                   'shoulder_pitch_link',
                   'upper_arm_link',
                   'elbow_link',
                   'fore_arm_link',
                   'wrist_yaw_link',
                   'wrist_pitch_link',
                   'wrist_palm_link',
                   'torque_sensor_link']

class_names_summit = ['wam\\viz']

joint_names = ['wam\\J1',
               'wam\\J2',
               'wam\\J3',
               'wam\\J4',
               'wam\\J5',
               'wam\\J6',
               'wam\\J7',
               'bhand\\f1\\prox',
               'bhand\\f1\\med',
               'bhand\\f1\\dist',
               'bhand\\f2\\prox',
               'bhand\\f2\\med',
               'bhand\\f2\\dist',
               'bhand\\f3\\med',
               'bhand\\f3\\dist']

mesh_file_names = ['mecanum_LF_1',
                   'mecanum_LR_1',
                   'mecanum_RF_1',
                   'mecanum_RR_1',
                   'base_link_fine',
                   'shoulder_link_fine',
                   'shoulder_pitch_link_fine',
                   'upper_arm_link_fine',
                   'elbow_link_fine',
                   'forearm_link_fine',
                   'wrist_yaw_link_fine',
                   'wrist_pitch_link_fine',
                   'wrist_palm_link_fine',
                   'torque_sensor_plate',
                   'base_link_convex',
                   'shoulder_link_convex_decomposition_p1',
                   'shoulder_link_convex_decomposition_p2',
                   'shoulder_link_convex_decomposition_p3',
                   'shoulder_pitch_link_convex',
                   'upper_arm_link_convex_decomposition_p1',
                   'upper_arm_link_convex_decomposition_p2',
                   'elbow_link_convex',
                   'forearm_link_convex_decomposition_p1',
                   'forearm_link_convex_decomposition_p2',
                   'wrist_yaw_link_convex_decomposition_p1',
                   'wrist_yaw_link_convex_decomposition_p2',
                   'wrist_pitch_link_convex_decomposition_p1',
                   'wrist_pitch_link_convex_decomposition_p2',
                   'wrist_pitch_link_convex_decomposition_p3',
                   'wrist_palm_link_convex',
                   'intel_realsense_l515',
                   'bhand_palm_fine',
                   'bhand_finger_prox_link_fine',
                   'bhand_finger_med_link_fine',
                   'bhand_finger_dist_link_fine',
                   'bhand_palm_link_convex_decomposition_p1',
                   'bhand_palm_link_convex_decomposition_p2',
                   'bhand_palm_link_convex_decomposition_p3',
                   'bhand_palm_link_convex_decomposition_p4',
                   'bhand_finger_prox_link_convex_decomposition_p1',
                   'bhand_finger_prox_link_convex_decomposition_p2',
                   'bhand_finger_prox_link_convex_decomposition_p3',
                   'bhand_finger_med_link_convex',
                   'bhand_finger_dist_link_convex']

# Set classes to one name
for geom in geoms:
  if type(geom.dclass) is str:
    # Check if geom has a class type having the sub-string `wam\\col` in it
    if 'wam\\\\col' in geom.dclass:
        geom.dclass = 'wam\\col'
    if 'wam\\bhand\\col' in geom.dclass:
        geom.dclass = 'wam\\bhand\\col'
    # Check if geom has a class type having the sub-string `summit\\col` in it
    if 'summit\\\\col' in geom.dclass:
        geom.dclass = 'summit\\col'
    # Check if geom has a class type having the sub-string `summit\\wheel\\viz` in it
    if 'summit\\\\wheel\\\\viz' in geom.dclass:
        geom.dclass = 'summit\\wheel\\viz'
    for class_name in reversed(class_names_bhand):
       if class_name in geom.dclass:
          geom.dclass = 'wam\\bhand\\'+class_name+'\\viz'
    for class_name in reversed(class_names_wam):
       if class_name in geom.dclass:
          geom.dclass = 'wam\\'+class_name+'\\viz'
    if '\\\\' in geom.dclass:
       temp_string = geom.dclass.replace('\\\\','\\')
       geom.dclass = temp_string
    for class_name in class_names_summit:
       if class_name in geom.dclass:
          geom.dclass = class_name
  # Check if geom has a mesh-type in it
  if geom.mesh is not None:
    for mesh_name in mesh_file_names:
      if mesh_name in geom.mesh:
          geom.mesh = mesh_name
  # Check if geom name has 'whl' in it
  if geom.name is not None:
     if 'whl' not in geom.name:
        del geom.name

# for geom in geoms:
#    print(geom.dclass)

# Remove class-attribute from 'joint' heading
for joint in joints:
   # Unite unnescessary class-names
   if type(joint.dclass) is str:
       if 'summit\\\\whl' in joint.dclass:
          joint.dclass = 'summit\\whl'
       else:
          joint.dclass = 'rand'
   if joint.name is not None:
      if '\\\\' in joint.name:
         temp_string = joint.name.replace('\\\\','\\')
         joint.name = temp_string
      for joint_name in joint_names:
         if joint_name in joint.name:
           joint.name = joint_name

temp_string = ""

# Replace '\\' instances in names with '\'
for body in bodies:
  if type(body.name) is str:
     if '\\\\' in body.name:
        temp_string = body.name.replace('\\\\','\\')
        body.name = temp_string

for camera in cameras:
  if type(camera.name) is str:
     print(camera.name)
     if '\\\\' in camera.name:
        temp_string = camera.name.replace('\\\\','\\')
        camera.name = temp_string
     for camera_name in camera_names:
        if camera_name in camera.name:
           camera.name = camera_name
  # Unite unnescessary class-names
  if type(camera.dclass) is str:
     camera.dclass = 'rand'
    #  if '\\\\' in camera.dclass:
    #     temp_string = camera.dclass.replace('\\\\','\\')
    #     camera.dclass = temp_string

for geom in geoms:
  if type(geom.name) is str:
     if '\\\\' in geom.name:
        temp_string = geom.name.replace('\\\\','\\')
        geom.name = temp_string
  if type(geom.dclass) is str:
     if '\\\\' in geom.dclass:
        temp_string = geom.dclass.replace('\\\\','\\')
        geom.dclass = temp_string

for joint in joints:
  if type(joint.name) is str:
     if '\\\\' in joint.name:
        temp_string = joint.name.replace('\\\\','\\')
        joint.name = temp_string
  
export_with_assets(mjcf_model = mjcf_model, out_dir=folder_name)
