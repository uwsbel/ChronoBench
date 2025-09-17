import pychrono
import pychrono.core as pc
import pychrono.visual as v
import pychrono.utils as uc
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.anchors import Anchor
from matplotlib.anchors.major import MajorAnchor
from pychrono.visual.mesh import Mesh
from pychrono.visual.camera import Camera
from pychrono.visual.ui import UI
from pychrono.visual.ui.ui import UI_Button
from pychrono.visual.ui.ui.ui import UI_Slider


class FEDAVehicle:
    def __init__(self, x, y, z, orientation, contact_method="friction", tire_model="rubber"):
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation  
        self.contact_method = contact_method
        self.tire_model = tire_model
        self.mass = 100.0  
        self.dynamic_friction = 0.1 
        self.acceleration = 0.0

    def update(self, dt):
        
        self.orientation += self.acceleration * dt
        self.orientation = np.clip(self.orientation, -np.pi, np.pi) 
        self.orientation = self.orientation % (2*np.pi) 

    def get_position(self):
        return (self.x, self.y, self.z)

    def get_orientation(self):
        return self.orientation

    def get_mass(self):
        return self.mass

    def get_dynamic_friction(self):
        return self.dynamic_friction


class RigidTerrain:
    def __init__(self, width, height, texture="grass"):
        self.width = width
        self.height = height
        self.texture = texture
        self.texture_data = np.zeros((height, width), dtype=np.uint8)

    def draw(self, vehicle):
        
        self.texture_data = np.array([
            [0, 0, 0],
            [0, 0, 1],
            [0, 1, 0],
            [0, 1, 1],
            [1, 1, 0],
            [1, 1, 1]
        ])
        self.texture_data = np.array(self.texture_data)



chrono = pychrono.core.Chrono(
    name="FEDA_Simulation",
    
    initial_time=0,
    initial_position=(0, 0, 0),
    initial_orientation=0,
    initial_mass=100.0,
    initial_dynamic_friction=0.1,
    initial_tire_model="rubber",
    
    camera_position=(0, 0, 0),
    camera_rotation=0,
    camera_scale=1.0
)


vehicle = FEDAVehicle(x=10, y=5, z=2, orientation=0, contact_method="friction", tire_model="rubber")


camera = Camera(position=(0, 0, 0), rotation=0, scale=1.0)


ui = UI_Button(
    position=(0, 0, 0),
    button_text="Steering",
    button_type="slider",
    button_scale=1.0,
    
    throttle_slider_position=(0, 100),
    throttle_slider_value=50,
    
    brake_slider_position=(0, 50),
    brake_slider_value=20
)


v.init(ui)


for frame in range(chrono.time):
    
    vehicle.update(chrono.time)

    
    v.render(vehicle, camera, ui)

    
    chrono.save(frame)

    
    print(f"Frame {frame}")


chrono.cleanup()