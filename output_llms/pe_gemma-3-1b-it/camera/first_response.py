import chrono
import pychrono
import numpy as np
import os
import math

# 1. Initialize PyChrono environment
try:
    chrono = chrono.ChSystem()
    chrono.SetChronoDataPath('/path/to/data') # Replace with your data directory
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    exit()

# 2. Import necessary modules
try:
    from pychrono.robot import RoboSimian
    from pychrono.irrlicht import Visualization
    from pychrono.vehicle import Vehicle
except ImportError as e:
    print(f"Error importing required modules: {e}")
    exit()


# 3. Define the simulation parameters
#   - Mesh:  Load a Wavefront .obj file
mesh_path = 'triangular_mesh.obj'  # Replace with your mesh file
try:
    mesh = RoboSimian(chrono, True, True, mesh_path)
    print(f"Mesh loaded successfully: {mesh.GetSystem()}")
except Exception as e:
    print(f"Error loading mesh: {e}")
    exit()

# 4. Set default parameters
#   - Collision System:  Set to 'BULLET' for a simple simulation
collision_system = chrono.ChCollisionSystem.Type_BULLET

#   - Default Contact Method:  'Euler' - Simple contact handling
contact_method = chrono.ChContactMethod.Type_Euler

#   - Default Contact Radius:  0.01
contact_radius = 0.01

#   - Default Damping: 0.000
damping = 0.000

#   - Default Compliance: 0.01
compliance = 0.01

#   - Default Simulation Time Step: 0.01
simulation_time_step = 0.01

#   - Default Camera Sensor:  Simple sensor with noise filters
camera_sensor = chrono.ChSensor()
camera_sensor.SetCamera(chrono.ChVector3d(0, 0, 0))
camera_sensor.SetNoiseFilters(chrono.ChNoiseFilter.Type_Linear)
camera_sensor.SetVisualization(chrono.ChVisualization.Type_Irrlicht)

# 5. Visualization Settings
try:
    visualization = Visualization(chrono, camera_sensor)
    print(f"Visualization initialized successfully: {visualization.GetSystem()}")
except Exception as e:
    print(f"Error initializing Visualization: {e}")
    exit()

# 6. Body Initialization
try:
    body = mesh.GetBody()
    print(f"Body initialized: {body.GetSystem()}")
except Exception as e:
    print(f"Error initializing body: {e}")
    exit()

# 7. Joint and Link Settings
try:
    # Revolute Joint
    rev_joint = body.GetJoint()
    rev_joint.SetRotation(chrono.ChVector3d(0, 0, 0)) # Initial rotation
    print(f"Revolute Joint initialized: {rev_joint.GetSystem()}")

    # Prismatic Joint
    prismatic_joint = body.GetJoint()
    prismatic_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Prismatic Joint initialized: {prismatic_joint.GetSystem()}")

    # Spherical Joint
    spherical_joint = body.GetJoint()
    spherical_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Spherical Joint initialized: {spherical_joint.GetSystem()}")

    # Universal Joint
    universal_joint = body.GetJoint()
    universal_joint.SetRotation(chrono.ChVector3d(0, 0, 0))
    print(f"Universal Joint initialized: {universal_joint.GetSystem()}")

    # 8.  Simulate the system
    print("Simulating the system...")
    sys.DoStepDynamics(simulation_time_step)

except Exception as e:
    print(f"Error during body initialization: {e}")
    exit()

# 9.  Visualization Loop
try:
    while visualization.Run():
        print(f"Visualization iteration: {visualization.GetIteration()}")
        # Simulate the system
        sys.DoStepDynamics(simulation_time_step)

except Exception as e:
    print(f"Error during visualization loop: {e}")
    exit()