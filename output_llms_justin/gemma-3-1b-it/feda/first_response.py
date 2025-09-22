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

# Define Physical Systems
class FEDAVehicle:
    def __init__(self, x, y, z, orientation, contact_method="friction", tire_model="rubber"):
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation  # Euler angles (roll, pitch, yaw)
        self.contact_method = contact_method
        self.tire_model = tire_model
        self.mass = 100.0  # kg
        self.dynamic_friction = 0.1 # kg*m*s^2
        self.acceleration = 0.0

    def update(self, dt):
        # Simplified dynamics - adjust for more realistic behavior
        self.orientation += self.acceleration * dt
        self.orientation = np.clip(self.orientation, -np.pi, np.pi) # Ensure within range
        self.orientation = self.orientation % (2*np.pi) # Normalize to 0-2pi

    def get_position(self):
        return (self.x, self.y, self.z)

    def get_orientation(self):
        return self.orientation

    def get_mass(self):
        return self.mass

    def get_dynamic_friction(self):
        return self.dynamic_friction

# Define Terrain
class RigidTerrain:
    def __init__(self, width, height, texture="grass"):
        self.width = width
        self.height = height
        self.texture = texture
        self.texture_data = np.zeros((height, width), dtype=np.uint8)

    def draw(self, vehicle):
        # Simple terrain drawing - can be customized
        self.texture_data = np.array([
            [0, 0, 0],
            [0, 0, 1],
            [0, 1, 0],
            [0, 1, 1],
            [1, 1, 0],
            [1, 1, 1]
        ])
        self.texture_data = np.array(self.texture_data)


# Initialize PyChrono
chrono = pychrono.core.Chrono(
    name="FEDA_Simulation",
    # Set initial parameters
    initial_time=0,
    initial_position=(0, 0, 0),
    initial_orientation=0,
    initial_mass=100.0,
    initial_dynamic_friction=0.1,
    initial_tire_model="rubber",
    # Set camera parameters
    camera_position=(0, 0, 0),
    camera_rotation=0,
    camera_scale=1.0
)

# Create Vehicle
vehicle = FEDAVehicle(x=10, y=5, z=2, orientation=0, contact_method="friction", tire_model="rubber")

# Create Camera
camera = Camera(position=(0, 0, 0), rotation=0, scale=1.0)

# Create UI
ui = UI_Button(
    position=(0, 0, 0),
    button_text="Steering",
    button_type="slider",
    button_scale=1.0,
    # Add a slider to control throttle
    throttle_slider_position=(0, 100),
    throttle_slider_value=50,
    # Add a slider to control braking
    brake_slider_position=(0, 50),
    brake_slider_value=20
)

# Initialize Visualization
v.init(ui)

# Simulation Loop
for frame in range(chrono.time):
    # Update Vehicle
    vehicle.update(chrono.time)

    # Render the Scene
    v.render(vehicle, camera, ui)

    # Save the Frame
    chrono.save(frame)

    # Print Frame Number (for debugging)
    print(f"Frame {frame}")

# Clean up
chrono.cleanup()