import os

from dm_control import mjcf
from dm_control.mujoco.wrapper import util

# Make a class for constructing legs
class Legs:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 0.4
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, height, radius, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # # Add some classes
        # leg_class = self.mjcf_model.default.add('default', dclass = 'leg_viz')
        # leg_class.site.set_attributes('geom' ,rgba = (0.95, 0.6, 0.05, 1))
        # Add a body element: No need to add position info, as that will be added based on which `site` this body is going to be attached (https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md#attachment-frames)
        self.leg_body = self.mjcf_model.worldbody.add('body',
                                                      name = name)
        # Add Inertial properties
        self.leg_body.add('inertial',pos = [0,0,0], mass = self._mass)
        # Add the Geometry (https://github.com/google-deepmind/dm_control/blob/ed424509c0a0e8ddf7a43824924de483026ad9cc/dm_control/locomotion/soccer/humanoid.py#L50)
        self.leg_body.add('geom',
                          name = name,
                          type = 'cylinder',
                          size = [radius, height/2],
                          contype = 0,
                          conaffinity = 0,
                          group = 1,
                          rgba = (0.95, 0.6, 0.05, 1))

class Shelf:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 1
    _rgba = (0.95, 0.6, 0.05, 1)
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, size, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # # Add some classes
        # leg_class = self.mjcf_model.default.add('default', dclass = 'rack_viz')
        # leg_class.site.set_attributes('geom' ,rgba = (0.95, 0.6, 0.05, 1))
        # Add a body element: No need to add position info, as that will be added based on which `site` this body is going to be attached (https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md#attachment-frames)
        self.shelf_body = self.mjcf_model.worldbody.add('body',
                                                        name = name)
        # Add Inertial properties
        self.shelf_body.add('inertial',pos = [0,0,0], mass = self._mass)
        # Add the Geometry
        self.shelf_body.add('geom',
                            name = name,
                            type = 'box',
                            size = size,
                            contype = 0,
                            conaffinity = 0,
                            group = 1,
                            rgba = self._rgba)
        
class Boxes:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 0.1
    _name = "boxes"
    _size = [0.05,0.05,0.05]
    _rgba = (0.6, 0.4, 0.3, 1)
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name, size=None):
        if(size!=None):
           # Replace the default size with the user-provided one
           self._size = size
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add a body element: No need to add position info, as that will be added based on which `site` this body is going to be attached (https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md#attachment-frames)
        self.shelf_body = self.mjcf_model.worldbody.add('body',
                                                        name = name)
        # Add Inertial properties
        self.shelf_body.add('inertial',pos = [0,0,0], mass = self._mass)
        # Add the Geometry
        self.shelf_body.add('geom',
                            name = name,
                            type = 'box',
                            size = self._size,
                            contype = 0,
                            conaffinity = 0,
                            group = 1,
                            rgba = self._rgba)

class Box_Collection:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _size_0 = [0.1,0.1,0.1]
    _size_1 = [0.2,0.3,0.2]
    _size_2 = [0.2,0.3,0.1]
    _size_3 = [0.1,0.3,0.25]
    _size_4 = [0.2,0.3,0.15]
    _size_0_name = 'box_0'
    _size_1_name = 'box_1'
    _size_2_name = 'box_2'
    _size_3_name = 'box_3'
    _size_4_name = 'box_4'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name):
        # Prepare a collection of boxes
        self.box_0_0 = Boxes(name = self._size_0_name+'_0', size = self._size_0)
        self.box_0_1 = Boxes(name = self._size_0_name+'_1', size = self._size_0)
        self.box_0_2 = Boxes(name = self._size_0_name+'_2', size = self._size_0)
        self.box_0_3 = Boxes(name = self._size_0_name+'_3', size = self._size_0)
        self.box_0_4 = Boxes(name = self._size_0_name+'_4', size = self._size_0)
        self.box_1 = Boxes(name = self._size_1_name, size = self._size_1)
        self.box_2 = Boxes(name = self._size_2_name, size = self._size_2)
        self.box_3_0 = Boxes(name = self._size_3_name+'_0', size = self._size_3)
        self.box_3_1 = Boxes(name = self._size_3_name+'_1', size = self._size_3)
        self.box_4 = Boxes(name = self._size_4_name, size = self._size_4)
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)

        # Prepare the arrangements of boxes for one shelf
        box_0_0 = self.mjcf_model.worldbody.add('site',
                                                pos = [-0.85, 0.2, 0.1])
        box_1_0 = self.mjcf_model.worldbody.add('site',
                                                pos = [-0.85, -0.2, 0.1])
        box_2_1 = self.mjcf_model.worldbody.add('site',
                                                pos = [-0.5, 0.2, 0.2])
        box_3_2 = self.mjcf_model.worldbody.add('site',
                                                pos = [-0.05, 0.1, 0.1])
        box_4_4 = self.mjcf_model.worldbody.add('site',
                                                pos = [-0.045, 0.15, 0.35])
        box_5_3 = self.mjcf_model.worldbody.add('site',
                                                pos = [0.3, 0.15, 0.25])
        box_6_3 = self.mjcf_model.worldbody.add('site',
                                                pos = [0.55, 0.1, 0.25])
        box_7_0 = self.mjcf_model.worldbody.add('site',
                                                pos = [0.8, 0.2, 0.1])
        box_8_0 = self.mjcf_model.worldbody.add('site',
                                                pos = [0.8, -0.05, 0.1])
        box_9_0 = self.mjcf_model.worldbody.add('site',
                                                pos = [0.8, 0.075, 0.3])
        # Attach the boxes
        box_0_0.attach(self.box_0_0.mjcf_model)
        box_1_0.attach(self.box_0_1.mjcf_model)
        box_2_1.attach(self.box_1.mjcf_model)
        box_3_2.attach(self.box_2.mjcf_model)
        box_4_4.attach(self.box_4.mjcf_model)
        box_5_3.attach(self.box_3_0.mjcf_model)
        box_6_3.attach(self.box_3_1.mjcf_model)
        box_7_0.attach(self.box_0_2.mjcf_model)
        box_8_0.attach(self.box_0_3.mjcf_model)
        box_9_0.attach(self.box_0_4.mjcf_model)

# Make a class for constructing shelves
class Shelves:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _name = "shelves"
    _size = [1.0,0.5,0.01]
    _shelf_height = 0.6
    _bottom_shelf_height = 0.05
    _leg_radius = 0.03
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, num_shelves, size=None):
        if(size!=None):
           # Replace the default size with the user-provided one
           self._size = size
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=self._name)
        
        # Create attachemnt-sites for adding 4 legs
        # Height
        shelf_total_height = (num_shelves-1)*self._shelf_height+self._bottom_shelf_height
        # Right-Top leg
        leg_rt = self.mjcf_model.worldbody.add('site',
                                              pos = [self._size[0],self._size[0],shelf_total_height/2])
        # Left-Top leg
        leg_lt = self.mjcf_model.worldbody.add('site',
                                              pos = [-self._size[0],self._size[0],shelf_total_height/2])
        # Left-Bottom leg
        leg_lb = self.mjcf_model.worldbody.add('site',
                                              pos = [-self._size[0],-self._size[0],shelf_total_height/2])
        # Right-Bottom leg
        leg_rb = self.mjcf_model.worldbody.add('site',
                                              pos = [self._size[0],-self._size[0],shelf_total_height/2])
        # Create leg-objects
        # Right-Top leg
        self.leg_rt = Legs(radius=self._leg_radius, height=shelf_total_height, name='leg_rt')
        # Left-Top leg
        self.leg_lt = Legs(radius=self._leg_radius, height=shelf_total_height, name='leg_lt')
        # Left-Bottom leg
        self.leg_lb = Legs(radius=self._leg_radius, height=shelf_total_height, name='leg_lb')
        # Right-Bottom leg
        self.leg_rb = Legs(radius=self._leg_radius, height=shelf_total_height, name='leg_rb')
        # Attach the leg
        # Right-Top leg
        leg_rt.attach(self.leg_rt.mjcf_model)
        # Left-Top leg
        leg_lt.attach(self.leg_lt.mjcf_model)
        # Left-Bottom leg
        leg_lb.attach(self.leg_lb.mjcf_model)
        # Right-Bottom leg
        leg_rb.attach(self.leg_rb.mjcf_model)
        
        # Create attachemnt-sites for adding shelves
        # Initilaize empty list of shelf-sites
        shelf_sites = []
        # Initilaize height variable
        curr_shelf_height = self._bottom_shelf_height
        for x in range(num_shelves):
            shelf_sites.append(self.mjcf_model.worldbody.add('site',
                                                            pos = [0,0,curr_shelf_height]))
            curr_shelf_height = curr_shelf_height+self._shelf_height
        # Create shelves
        # Initialize empty list of shelves
        shelf = []
        for x in range(num_shelves):
            shelf.append(Shelf(self._size, 'shelf_'+str(x)))
        # Attach the shelf
        for x in range(num_shelves):
            shelf_sites[x].attach(shelf[x].mjcf_model)
        
        # Create Box-collections
        # Initialize empty list of box-collections
        box_col = []
        for x in range(num_shelves):
            box_col.append(Box_Collection('boc_col_'+str(x)))
        # Attach box-collections to each shelf-site
        for x in range(num_shelves):
            shelf_sites[x].attach(box_col[x].mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.mjcf_model.worldbody.site.clear()

        # Find all classes
        # Unable to remove it. For an empty class, since the script enters `<default class="leg_rt/"/>`, which 
        # cannot be interpreted b MuJoCo, we will need to modify it as such `<default class="leg_rt/"></default>`
        # classes = self.mjcf_model.find_all('geom')
        # print(classes[0].name)
        # print(classes[0].dclass)
        # del classes[0].dclass

# class Environment:
#    #===================#
#     #  C O N S T A N T  #
#     #===================#
#     _name = "shelves"
#     _size = [1.0,0.5,0.01]
#     _shelf_height = 0.6
#     _bottom_shelf_height = 0.05
#     _leg_radius = 0.03
#     #===============================#
#     #  I N I T I A L I Z A T I O N  #
#     #===============================#
#     def __init__(self, num_shelves, size=None):

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

# Create a shelf
shelf_body = Shelves(6)
export_with_assets(mjcf_model = shelf_body.mjcf_model, out_dir='/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/src/out')