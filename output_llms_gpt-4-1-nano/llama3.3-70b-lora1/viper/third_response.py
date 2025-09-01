import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
from pychrono import irrlicht as chronoirr

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

#  Create the SCM terrain, a rigid terrain with a decentralized module
terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100, 100)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
terrain.Initialize()

# Create Viper rover
rover = robot.Viper(system)  # Instantiate the Viper rover
driver = robot.ViperDCMotorControl()  # Create a driver for the rover
rover.SetDriver(driver)  # Assign the driver to the rover

# Initialize rover position and orientation
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Viper')
vis.SetWindowSize(1280, 720)
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 0.75), 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(rover.GetVehicle())

# Simulation loop
steering = 0
steering_angle = 0

while vis.Run() :
    time = rover.GetSystem().GetChTime()

    # Driver inputs
    if (time < 2) :
        motor_power = 0.0
    else :
        motor_power = 0.8

    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation of the rover for one timestep
    rover.Advance(step_size)
    vis.Advance(step_size)

    # Update modules (process inputs from other modules)
    driver_inputs = veh.DriverInputs()
    driver_inputs.m_steering = 0
    driver_inputs.m_throttle = 0.1
    driver_inputs.m_braking = 0.0
    driver.Synchronize(time, driver_inputs)
    rover.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)