import chrono
import numpy as np
import math
import os
import sys

# Initialize PyChrono environment
try:
    chrono = chrono.ChSystem()
    chrono.ChSystemNSC()
    chrono.ChSystemSMC()
    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
    chrono.ChContactMaterialNSC()
except Exception as e:
    print(f"Error initializing Chrono: {e}")
    sys.exit(1)

# --- Define the Simulation Parameters ---
# Define the terrain geometry
terrain_length = 100
terrain_width = 100
terrain_height = 5
terrain_x = 0
terrain_y = 0
terrain_z = 0

# Define the rover's initial position
rover_x = 10
rover_y = 10
rover_z = 10

# Define the rover's initial orientation (rotation)
rover_rotation = chrono.ChVector3d(0, 0, 0)

# Define the motor control driver
motor_driver = chrono.ChLinkLockLock()
motor_driver.SetMotionY(chrono.ChFunctionSine(0.001, 1.5))

# --- Create the Physical System ---
# Create the Rigid Terrain
terrain = chrono.ChRigidTerrain(terrain_length, terrain_width, terrain_height)

# --- Create the Rover ---
rover = chrono.ChBody()
rover.SetPos(chrono.ChVector3d(rover_x, rover_y, rover_z))
rover.SetMass(100)
rover.SetInertiaXX(chrono.ChVector3d(10, 10, 10))
rover.SetRayleighDamping(0.000)
rover.SetContactMethod(chrono.ChContactMethod.Type_BULLET)
rover.SetChassisCollisionType(chrono.ChCollisionSystem.Type_BULLET)

# --- Add the Rover to the System ---
sys.Add(rover)

# --- Visualize the Simulation ---
# Create the Irrlicht visualization system
vis = chrono.ChVisualSystemIrrlicht()

# Set window size
vis.SetWindowSize(1024, 768)

# --- Set Camera Settings ---
# Camera position
camera_x = 10
camera_y = 10
camera_z = 10

# Set camera color
vis.SetWindowTitle('Curiosity Rover Simulation')
vis.SetCamera(chrono.ChCoordsysd(chrono.ChVector3d(camera_x, camera_y, camera_z)))

# --- Run the Simulation ---
# Run the simulation
sys.DoStepDynamics(0.01)

# --- Display the Simulation ---
# Display the simulation
vis.Render()

# --- End of Simulation ---