import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.robot as rob
import pychrono.irrlicht as chronoirr

# Initialize PyChrono data path
chrono.SetChronoDataPath(chrono.GetChronoDataFile(''))

# Create simulation system
sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gator Vehicle Simulation')
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()

# Create Gator vehicle
gator = veh.Gator(sys, veh.Gator_VEHICLE_TYPE::GATOR_V4, True, True)
gator.SetContactMethod(veh.ContactMethod::NSC)
gator.SetChassisCollisionType(veh.ChassisCollisionType::NONE)
gator.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
gator.Initialize()

# Set visualization types for different parts
gator.GetChassis().GetVisualShape(0).SetVisualizationType(chrono.VisualizationType::CHRONO)
gator.GetWheel(0).GetVisualShape(0).SetVisualizationType(chrono.VisualizationType::NONE)
gator.GetSuspension(0).GetVisualShape(0).SetVisualizationType(chrono.VisualizationType::PRIMITIVES)

# Create interactive driver
driver = rob.ChInteractiveDriver()
driver.AttachToVehicle(gator.GetDriverInputs())
sys.Add(driver)

# Create terrain
terrain = veh.RigidTerrain(sys)
patch_mat = veh.ChMaterialSurface()
patch_mat.SetFriction(0.9)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Create sensor manager
sensor_manager = veh.SensorManager(sys)
camera = sensor_manager.AttachCameraToChassis(gator.GetChassis(), chrono.ChVectorD(1, 0, 1), chrono.ChVectorD(0, 1, 0))
sensor_manager.AddPointLight(chrono.ChVectorD(2, 0, 2), chrono.ChColor(1, 1, 1))
sensor_manager.AddPointLight(chrono.ChVectorD(-2, 0, 2), chrono.ChColor(1, 1, 1))

# Add vehicle and terrain to visualization
vis.Add(gator.GetChassis())
vis.Add(terrain.GetGround())

# Simulation loop
while vis.Run():
    vis.BeginScene()
    
    # Update driver
    driver.Update()
    
    # Update terrain
    terrain.Update()
    
    # Update vehicle
    gator.Update()
    
    # Update sensor manager
    sensor_manager.Update()
    
    # Render scene
    vis.Render()
    
    # Synchronize vehicle systems
    gator.Synchronize()
    
    # Advance simulation
    sys.DoStepDynamics(0.01)
    
    vis.EndScene()

# Cleanup
sensor_manager.Close()