import chrono
import pychrono
import math
import numpy as np
import os
import sys
import time
import random

# Initialize PyChrono environment
try:
    chrono = chrono.ChSystem()
    chrono.ChSystemNSC()
    chrono.ChSystemSMC()
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    sys.exit(1)

# Define the physical system
try:
    my_system = chrono.ChBody()
except Exception as e:
    print(f"Error creating physical system: {e}")
    sys.exit(1)

# Define the rigid terrain
try:
    terrain = chrono.ChRigidTerrain()
except Exception as e:
    print(f"Error creating rigid terrain: {e}")
    sys.exit(1)

# Define the texture
try:
    texture_path = os.path.join(chrono.GetChronoDataFile('texture_pychrono_alpha.png'), 'texture')
    terrain.AddTexture(texture_path)
except Exception as e:
    print(f"Error adding texture: {e}")
    sys.exit(1)

# Define the vehicle parameters
try:
    vehicle_type = "ARTcar"
    vehicle_x = 0
    vehicle_y = 0
    vehicle_z = 0
    vehicle_radius = 1.0
    vehicle_mass = 100
    vehicle_length = 2.0
    vehicle_width = 1.0
    vehicle_height = 0.5
    vehicle_material = "Steel"
    vehicle_friction = 0.5
    vehicle_damping = 0.01
    vehicle_rayleigh_dampening = 0.000
    vehicle_speed = 0.0

except Exception as e:
    print(f"Error defining vehicle parameters: {e}")
    sys.exit(1)


# Define the collision method
try:
    collision_method = "Standard"
except Exception as e:
    print(f"Error defining collision method: {e}")
    sys.exit(1)

# Define the simulation loop
try:
    while True:
        # Simulate the vehicle dynamics
        vehicle_velocity = np.array([vehicle_x, vehicle_y, vehicle_z])
        vehicle_acceleration = np.array([0.0, 0.0, 0.0])
        vehicle_angular_velocity = np.array([0.0, 0.0, 0.0])

        # Update vehicle dynamics
        vehicle_velocity = vehicle_velocity + vehicle_acceleration * 0.1
        vehicle_angular_velocity = vehicle_angular_velocity + vehicle_acceleration * 0.1

        # Apply contact
        if collision_method == "Standard":
            contact_force = chrono.ChContactForce()
            contact_force.SetForce(vehicle_velocity)
            contact_force.SetDirection(chrono.ChVector3d(0, 0, 0))
        elif collision_method == "Bullet":
            contact_force = chrono.ChContactForce()
            contact_force.SetForce(vehicle_velocity)
            contact_force.SetDirection(chrono.ChVector3d(0, 0, 0))
        elif collision_method == "Rayleigh":
            contact_force = chrono.ChContactForce()
            contact_force.SetForce(vehicle_velocity)
            contact_force.SetDirection(chrono.ChVector3d(0, 0, 0))
        else:
            print("Invalid collision method specified.")
            sys.exit(1)

        # Update the vehicle's position
        vehicle_x = vehicle_x + vehicle_velocity * 0.1
        vehicle_y = vehicle_y + vehicle_velocity * 0.1
        vehicle_z = vehicle_z + vehicle_velocity * 0.1

        # Update the vehicle's orientation
        vehicle_angular_velocity = vehicle_angular_velocity + vehicle_acceleration * 0.1

        # Update the vehicle's position
        vehicle_x = vehicle_x + vehicle_velocity * 0.1
        vehicle_y = vehicle_y + vehicle_velocity * 0.1
        vehicle_z = vehicle_z + vehicle_velocity * 0.1

        # Display the simulation
        vis = chrono.ChVisualSystemIrrlicht()
        vis.AttachSystem(my_system)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle('ARTcar Simulation')
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'), chrono.ChCoordsysD(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0)))
        vis.AddCamera(chrono.ChVector3d(0, 0, 0))
        vis.Render()
        time.sleep(0.01)  # Add a small delay for visualization

        # Print simulation progress
        print(f"Time: {time.time()}")
        print("Simulation running...")

except Exception as e:
    print(f"Error during simulation: {e}")
    sys.exit(1)