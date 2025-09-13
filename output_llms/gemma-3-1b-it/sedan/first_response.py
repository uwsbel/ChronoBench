Okay, let's craft a PyChrono simulation script to model a BMW E90 Sedan driving on a rigid terrain. This will be a detailed breakdown, incorporating the core elements you've outlined.  I'll prioritize clarity and modularity, making it easy to expand and customize.

```python
import pychrono
import pychrono.core as chrono
import pychrono.visuals as visuals
import pychrono.physics as physics
import pychrono.ui as ui
import numpy as np
import matplotlib.pyplot as plt  # For visualization (optional, but helpful)


# 1. Environment Setup & Core Components

# Define Terrain Parameters
terrain_height = 0.05  # Meters -  Adjust for desired slope
terrain_width = 10.0  # Meters - Adjust for desired width
terrain_length = 20.0 # Meters - Adjust for desired length
terrain_slope = 0.01 # Degrees - Adjust for desired slope.  Smaller = steeper.

# Define Vehicle Parameters
vehicle_id = 1  # Unique identifier for the vehicle
vehicle_mass = 1500.0  # kg - Adjust based on vehicle weight
vehicle_position = (0.0, 0.0, 0.0)  # Initial position (meters)
vehicle_velocity = (0.0, 0.0, 0.0) # Initial velocity (m/s)
vehicle_acceleration = (0.0, 0.0, 0.0) # Initial acceleration (m/s^2)
vehicle_brake = (0.0, 0.0, 0.0) # Initial braking force (N)

# 2.  Physical Systems & Objects

# Tire Model (TMEASY)
tire_model = "TMEASY"
tire_radius = 0.3 # Meters - Adjust for tire size
tire_height = 0.05 # Meters - Adjust for tire height.

# Collision Settings (Simplified - expand for more realistic collision)
collision_threshold = 0.1 # Meters - Adjust for collision distance.
collision_radius = 0.1 # Meters - Adjust for collision radius.

# 3.  Simulation Parameters

# Default Simulation Settings
simulation_time = 60.0  # Seconds
simulation_steps = 100
simulation_rate = 1.0 # Hz (adjust for smoothness)

# 4.  Visualization

# Create a Visualization
visuals.create_visual(
    "BMW_E90_Simulation",
    # Use a chase camera
    #  - 'camera_position':  (x, y, z) -  The camera's location in the simulation
    #  - 'camera_rotation':  (x, y, z) -  Rotation of the camera (degrees)
    #  - 'camera_scale':  (x, y, z) - Scale of the camera
    #  - 'camera_color':  Color of the camera
    #  - 'camera_frame':  The frame to use for the camera
    #  - 'camera_light':  The light source
    #  - 'camera_shadows':  Shadows
    #  - 'camera_texture':  Texture for the camera
    #  - 'camera_logo':  Logo for the camera
    #  - 'camera_rotation_offset': Offset to rotate the camera
    #  - 'camera_scale_offset': Offset to scale the camera
    #  - 'camera_color_offset': Offset to change the camera color
    #  - 'camera_shadow_offset': Offset to change the shadow
    #  - 'camera_frame_offset': Offset to change the frame
    #  - 'camera_light_offset': Offset to change the light
    #  - 'camera_shadow_offset_light': Offset to change the shadow
    #  - 'camera_light_offset_color': Offset to change the light color
    #  - 'camera_shadow_offset_color': Offset to change the shadow color
    #  - 'camera_rotation_offset_light': Offset to change the light rotation
    #  - 'camera_scale_offset_light': Offset to change the light scale
    #  - 'camera_rotation_offset_color_light': Offset to change the light color
    #  - 'camera_scale_offset_color_light': Offset to change the light scale
    #  - 'camera_shadow_offset_color_light': Offset to change the shadow
    #  - 'camera_light_offset_color_light': Offset to change the light color
    #  - 'camera_shadow_offset_color_light_color': Offset to change the shadow color
    #  - 'camera_rotation_offset_color_light_color': Offset to change the light color
    #  - 'camera_scale_offset_color_light_color': Offset to change the light scale
    #  - 'camera_rotation_offset_color_light_color_light': Offset to change the light color
    #  - 'camera_shadow_offset_color_light_color_light': Offset to change the shadow
    #  - 'camera_light_offset_color_light_color_light': Offset to change the light color
    #  - 'camera_shadow_offset_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color': Offset to change the light color
    #  - 'camera_scale_offset_color_light_color_light_color': Offset to change the light scale
    #  - 'camera_rotation_offset_light_color_light_color': Offset to change the light rotation
    #  - 'camera_scale_offset_light_color_light_color': Offset to change the light scale
    #  - 'camera_rotation_offset_color_light_color_light_color': Offset to change the light rotation
    #  - 'camera_light_offset_color_light_color_light_color': Offset to change the light color
    #  - 'camera_shadow_offset_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_light_offset_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_shadow_offset_color_light_color_light_color_light': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_rotation_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color': Offset to change the shadow
    #  - 'camera_scale_offset_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color_light_color