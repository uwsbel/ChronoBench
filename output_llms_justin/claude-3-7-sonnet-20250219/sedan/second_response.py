import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import math
import numpy as np

"""
!!!! Set this path before running the demo!
"""
chrono.SetChronoDataPath(chrono.GetChronoDataPath())
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle locations and orientations
initLoc1 = chrono.ChVector3d(0, 0, 0.5)
initRot1 = chrono.ChQuaterniond(1, 0, 0, 0)

# Second vehicle position and orientation
initLoc2 = chrono.ChVector3d(5, -5, 0.5)
initRot2 = chrono.ChQuaterniond(1, 0, 0, 0)

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
vis_type = veh.VisualizationType_MESH

# Collision type for chassis (PRIMITIVES, MESH, or NONE)
chassis_collision_type = veh.CollisionType_NONE

# Type of tire model (RIGID, TMEASY)
tire_model = veh.TireModelType_TMEASY

# Rigid terrain
terrainHeight = 0      # terrain height
terrainLength = 100.0  # size in X direction
terrainWidth = 100.0   # size in Y direction

# Points tracked by the camera
trackPoint1 = chrono.ChVector3d(-5.0, 0.0, 1.8)
trackPoint2 = chrono.ChVector3d(-5.0, 0.0, 1.8)

# Contact method
contact_method = chrono.ChContactMethod_NSC
contact_vis = False

# Simulation step sizes
step_size = 1e-3
tire_step_size = step_size

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# --------------
# Create systems
# --------------

# Create the first Sedan vehicle, set parameters, and initialize
vehicle1 = veh.BMW_E90()
vehicle1.SetContactMethod(contact_method)
vehicle1.SetChassisCollisionType(chassis_collision_type)
vehicle1.SetChassisFixed(False)
vehicle1.SetInitPosition(chrono.ChCoordsysd(initLoc1, initRot1))
vehicle1.SetTireType(tire_model)
vehicle1.SetTireStepSize(tire_step_size)
vehicle1.Initialize()
vehicle1.SetChassisVisualizationType(vis_type)
vehicle1.SetSuspensionVisualizationType(vis_type)
vehicle1.SetSteeringVisualizationType(vis_type)
vehicle1.SetWheelVisualizationType(vis_type)
vehicle1.SetTireVisualizationType(vis_type)

# Create the second Sedan vehicle, set parameters, and initialize
vehicle2 = veh.BMW_E90()
vehicle2.SetContactMethod(contact_method)
vehicle2.SetChassisCollisionType(chassis_collision_type)
vehicle2.SetChassisFixed(False)
vehicle2.SetInitPosition(chrono.ChCoordsysd(initLoc2, initRot2))
vehicle2.SetTireType(tire_model)
vehicle2.SetTireStepSize(tire_step_size)
vehicle2.Initialize()
vehicle2.SetChassisVisualizationType(vis_type)
vehicle2.SetSuspensionVisualizationType(vis_type)
vehicle2.SetSteeringVisualizationType(vis_type)
vehicle2.SetWheelVisualizationType(vis_type)
vehicle2.SetTireVisualizationType(vis_type)

# Set collision system type for both vehicles
vehicle1.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
vehicle2.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the terrain for vehicle 1 (each vehicle has its own system)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)

terrain1 = veh.RigidTerrain(vehicle1.GetSystem())
patch1 = terrain1.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Changed texture from tile4.jpg to concrete.jpg
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain1.Initialize()

# Create the terrain for vehicle 2
terrain2 = veh.RigidTerrain(vehicle2.GetSystem())
patch2 = terrain2.AddPatch(patch_mat, 
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
    terrainLength, terrainWidth)

# Using the same concrete texture for second terrain
patch2.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch2.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain2.Initialize()

# Create the vehicle Irrlicht interface
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('Multiple Sedans')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint1, 6.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()
vis.AddSkyBox()
vis.AttachVehicle(vehicle1.GetVehicle())

# Create the driver systems
driver1 = veh.ChInteractiveDriverIRR(vis)
driver2 = veh.ChDriver(vehicle2.GetVehicle())  # Non-interactive driver for vehicle2

# Set the time response for steering and throttle keyboard inputs.
steering_time = 1.0  # time to go from 0 to +1 (or from 0 to -1)
throttle_time = 1.0  # time to go from 0 to +1
braking_time = 0.3   # time to go from 0 to +1
driver1.SetSteeringDelta(render_step_size / steering_time)
driver1.SetThrottleDelta(render_step_size / throttle_time)
driver1.SetBrakingDelta(render_step_size / braking_time)

driver1.Initialize()
driver2.Initialize()

# ---------------
# Simulation loop
# ---------------

# output vehicle masses
print("VEHICLE 1 MASS: ", vehicle1.GetVehicle().GetMass())
print("VEHICLE 2 MASS: ", vehicle2.GetVehicle().GetMass())

# Number of simulation steps between miscellaneous events
render_steps = math.ceil(render_step_size / step_size)

# Initialize simulation frame counter
realtime_timer = chrono.ChRealtimeStepTimer()
step_number = 0
render_frame = 0

while vis.Run():
    time = vehicle1.GetSystem().GetChTime()

    # Render scene
    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    # Get driver inputs for vehicle 1
    driver_inputs1 = driver1.GetInputs()
    
    # Create sinusoidal steering inputs for both vehicles
    # For vehicle 1, combine manual input with sine wave
    sine_steering1 = 0.2 * math.sin(0.5 * time)
    driver_inputs1.m_steering += sine_steering1
    
    # For vehicle 2, create automated sine wave steering and constant throttle
    sine_steering2 = 0.4 * math.sin(0.3 * time)
    driver_inputs2 = veh.DriverInputs()
    driver_inputs2.m_steering = sine_steering2
    driver_inputs2.m_throttle = 0.3  # Constant throttle for vehicle 2
    driver_inputs2.m_braking = 0.0
    
    # Update modules for vehicle 1
    driver1.Synchronize(time)
    terrain1.Synchronize(time)
    vehicle1.Synchronize(time, driver_inputs1, terrain1)
    vis.Synchronize(time, driver_inputs1)
    
    # Update modules for vehicle 2
    driver2.Synchronize(time, driver_inputs2)
    terrain2.Synchronize(time)
    vehicle2.Synchronize(time, driver_inputs2, terrain2)
    
    # Advance simulation for one timestep for all modules
    driver1.Advance(step_size)
    terrain1.Advance(step_size)
    vehicle1.Advance(step_size)
    
    driver2.Advance(step_size)
    terrain2.Advance(step_size)
    vehicle2.Advance(step_size)
    
    vis.Advance(step_size)
    
    # Increment frame number
    step_number += 1

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)