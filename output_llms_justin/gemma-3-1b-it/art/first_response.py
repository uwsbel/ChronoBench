import pychrono
import pychrono.core as pc
import pychrono.visuals as v
import pychrono.physics as pcf
import pychrono.utils as uc
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.anchors import Anchor
from matplotlib.animation import FuncAnimation

# --- Simulation Parameters ---
terrain_width = 10.0
terrain_height = 5.0
terrain_depth = 2.0
vehicle_x = 0.0
vehicle_y = 0.0
vehicle_z = 0.0
vehicle_speed = 0.5
vehicle_acceleration = 0.1
vehicle_deceleration = 0.1
vehicle_max_speed = 5.0
vehicle_max_acceleration = 2.0
vehicle_max_deceleration = 1.0
vehicle_contact_method = 'rigid'  # 'rigid', 'flexible'
vehicle_visibility = 1.0
vehicle_texture = 'texture.png' # Replace with your texture file
simulation_duration = 60.0  # Simulation duration in seconds

# --- Initial Setup ---
try:
    # Initialize PyChrono
    chrono = pc.chrono.Chronos()
    chrono.init()

    # Create a RigidTerrain object
    rigid_terrain = v.RigidTerrain(
        width=terrain_width,
        height=terrain_height,
        depth=terrain_depth,
        texture=uc.Texture(file=vehicle_texture),
        # Add a simple ground plane for better visualization
        ground_plane=v.GroundPlane(
            x=0.0,
            y=0.0,
            z=terrain_height,
            visible=True
        )
    )

    # Create a Vehicle object
    vehicle = v.Vehicle(
        x=vehicle_x,
        y=vehicle_y,
        z=vehicle_z,
        speed=vehicle_speed,
        acceleration=vehicle_acceleration,
        deceleration=vehicle_deceleration,
        contact_method=vehicle_contact_method,
        visibility=vehicle_visibility,
        texture=vehicle_texture
    )

    # Set up the vehicle's orientation
    vehicle.set_orientation(anchor=Anchor(x=0.0, y=0.0, z=0.0))

    # --- Simulation Loop ---
    def update_simulation(vehicle):
        """Updates the vehicle's position and orientation based on physics."""
        vehicle.update_dynamics()

        # Update the vehicle's position based on velocity
        vehicle.update_position(vehicle.speed * dt)

        # Update the vehicle's orientation based on acceleration
        vehicle.update_orientation(vehicle.acceleration * dt)

        # Update the visualization
        v.update(vehicle)

    dt = 1.0 / simulation_duration
    for _ in range(simulation_duration):
        update_simulation(vehicle)

    # --- Visualization ---
    fig, ax = plt.subplots()
    ax.set_xlim(0, terrain_width)
    ax.set_ylim(0, terrain_height)
    ax.set_aspect('equal')
    ax.set_title('ARTcar Simulation')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create a FuncAnimation object
    ani = FuncAnimation(fig, update_simulation, interval=50, blit=True)

    # Display the animation
    plt.show()

except Exception as e:
    print(f"An error occurred: {e}")
    print("Please ensure that PyChrono is installed correctly and that the terrain and vehicle are properly defined.")