import os

from dm_control import mjcf
from dm_control.mujoco.wrapper import util

# Make a class for constructing legs
class Legs:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 0.4
    _rgba = (0.8, 0.35, 0.1, 1)
    _material = 'orange_metal'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, height, radius, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add some classes
        leg_class = self.mjcf_model.default.add('default', dclass = 'leg_viz')
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
                          rgba = self._rgba,
                          dclass = 'leg_viz')

class Shelf:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 1
    _rgba = (0.8, 0.35, 0.1, 1)
    _material = 'orange_metal'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, size, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add some classes
        shelf_class = self.mjcf_model.default.add('default', dclass = 'rack_viz')
        # shelf_class.site.set_attributes('geom' ,rgba = self._rgba)
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
                            rgba = self._rgba,
                            dclass = 'rack_viz')
        
class Boxes:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _mass = 0.1
    _name = "boxes"
    _size = [0.05,0.05,0.05]
    _material = 'cardboard'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name, size=None):
        if(size!=None):
           # Replace the default size with the user-provided one
           self._size = size
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add some classes
        boxes_class = self.mjcf_model.default.add('default', dclass = 'box_viz')
        # boxes_class.site.set_attributes('geom' ,rgba = self._rgba)
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
                            dclass = 'box_viz',
                            material = self._material)

class Box_Collection:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _size_0 = [0.2,0.4,0.25]
    _location = [[0.8,0.05,0.25],
                 [0.4,-0.05,0.25],
                 [0,0,0.25],
                 [-0.4,0.05,0.25],
                 [-0.8,-0.05,0.25]]
    _site_name = 'site'
    _size_0_name = 'box'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=name)
        # Add some classes
        box_col_class = self.mjcf_model.default.add('default', dclass = 'box_col_viz')

        # Prepare the arrangements of boxes for one shelf
        # An empty array to hold box-sites
        box_site = []
        for x in range(len(self._location)):
            box_site.append(self.mjcf_model.worldbody.add('site',
                                                          pos = self._location[x],
                                                          dclass = self._site_name))
        # Create boxes
        # Initialize empty list of boxes
        boxes = []
        for x in range(len(self._location)):
            boxes.append(Boxes(name = self._size_0_name+str(x), size = self._size_0))
        # Attach the boxes
        for x in range(len(self._location)):
            box_site[x].attach(boxes[x].mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.mjcf_model.worldbody.site.clear()

# Make a class for constructing shelves
class Shelves:
    #===================#
    #  C O N S T A N T  #
    #===================#
    _name = 'shelves'
    _site_name = 'site'
    _size = [1.0,0.5,0.01]
    _shelf_height = 0.6
    _bottom_shelf_height = 0.05
    _leg_radius = 0.03
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, num_shelves, size=None, name = None):
        if(size!=None):
           # Replace the default size with the user-provided one
           self._size = size
        if(name!=None):
           # Replace the default name with the user-provided one
           self._name = name
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=self._name)
        # Add some classes
        shelves_class = self.mjcf_model.default.add('default', dclass = 'shelves_viz')

        # Create attachemnt-sites for adding 4 legs
        # Height
        shelf_total_height = (num_shelves-1)*self._shelf_height+self._bottom_shelf_height
        # Right-Top leg
        leg_rt = self.mjcf_model.worldbody.add('site',
                                               name = self._site_name+'_rt',
                                               pos = [self._size[0]-self._leg_radius*2,self._size[1]-self._leg_radius*2,shelf_total_height/2],
                                               dclass = self._site_name)
        # Left-Top leg
        leg_lt = self.mjcf_model.worldbody.add('site',
                                               name = self._site_name+'_lt',
                                               pos = [-(self._size[0]-self._leg_radius*2),self._size[1]-self._leg_radius*2,shelf_total_height/2],
                                               dclass = self._site_name)
        # Left-Bottom leg
        leg_lb = self.mjcf_model.worldbody.add('site',
                                               name = self._site_name+'_lb',
                                               pos = [-(self._size[0]-self._leg_radius*2),-(self._size[1]-self._leg_radius*2),shelf_total_height/2],
                                               dclass = self._site_name)
        # Right-Bottom leg
        leg_rb = self.mjcf_model.worldbody.add('site',
                                               name = self._site_name+'_rb',
                                               pos = [self._size[0]-self._leg_radius*2,-(self._size[1]-self._leg_radius*2),shelf_total_height/2],
                                               dclass = self._site_name)
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
                                                             pos = [0,0,curr_shelf_height],
                                                             dclass = self._site_name))
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
            box_col.append(Box_Collection('box_col_'+str(x)))
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

class Shelf_Wall:
   #===================#
    #  C O N S T A N T  #
    #===================#
    _shelf_locations = [[-4,0,0],
                        [-2,0,0],
                        [0,0,0],
                        [2,0,0],
                        [4,0,0]]
    _num_racks = 6
    _name = 'shelf_wall'
    _site_name = 'site'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self, name):
        if(name!=None):
           # Replace the default name with the user-provided one
           self._name = name
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=self._name)

        # Set-up at all the locations
        # Create attachemnt-sites for adding a shelf
        # Initilaize empty list of shelf-sites
        shelf_sites = []
        for x in range(len(self._shelf_locations)):
            shelf_sites.append(self.mjcf_model.worldbody.add('site',
                                                             pos = self._shelf_locations[x],
                                                             dclass = self._site_name))
        # Create shelves
        # Initialize empty list of shelves
        shelves = []
        for x in range(len(self._shelf_locations)):
            shelves.append(Shelves(num_shelves = self._num_racks, name = 'shelves_'+str(x)))
        # Attach the shelf
        for x in range(len(self._shelf_locations)):
            shelf_sites[x].attach(shelves[x].mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.mjcf_model.worldbody.site.clear()

class Environment:
   #===================#
    #  C O N S T A N T  #
    #===================#
    _shelf_locations = [[-14,0,0],
                        [0,0,0],
                        [14,0,0],
                        [-14,4,0],
                        [0,4,0],
                        [14,4,0]]
    _name = 'shelf_env'
    _site_name = 'site'
    #===============================#
    #  I N I T I A L I Z A T I O N  #
    #===============================#
    def __init__(self):
        # Make a MujoCo Root
        self.mjcf_model = mjcf.RootElement(model=self._name)

        # Set-up at all the locations
        # Create attachemnt-sites for adding a shelf-wall
        # Initilaize empty list of shelf-wall-sites
        shelf_wall_sites = []
        for x in range(len(self._shelf_locations)):
            shelf_wall_sites.append(self.mjcf_model.worldbody.add('site',
                                                                  pos = self._shelf_locations[x],
                                                                  dclass = self._site_name))
        # Create shelves
        # Initialize empty list of shelves
        shelf_wall = []
        for x in range(len(self._shelf_locations)):
            shelf_wall.append(Shelf_Wall(name = 'shelf_wall_'+str(x)))
        # Attach the shelf
        for x in range(len(self._shelf_locations)):
            shelf_wall_sites[x].attach(shelf_wall[x].mjcf_model)

        # Delete attachment frames. The element `site` is being read by MuJoCo and, since it does not recongnize this element, it throws an error.
        self.mjcf_model.worldbody.site.clear()

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
# shelf_body = Shelves(6)
env_body = Environment()
folder_name = '/home/arnab/UWARL_catkin_ws/src/uwarl-mujoco-summit-wam-sim/src/out'
export_with_assets(mjcf_model = env_body.mjcf_model, out_dir=folder_name)

# Read the created file
filename = folder_name+'/shelf_env.xml'
mjcf_model = mjcf.from_path(filename, escape_separators=True)

geoms = mjcf_model.find_all('geom')
# print(classes[0].name)
# print(classes[0].dclass)

# Set classes to one name
for geom in geoms:
  geom.dclass = 'default'
  # Check if geom has a material type with `cardboard` in it
  if geom.material is not None:
    if 'cardboard' in geom.material:
        geom.material = 'cardboard'
  
export_with_assets(mjcf_model = mjcf_model, out_dir=folder_name)