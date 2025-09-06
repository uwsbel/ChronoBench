# Import necessary libraries
import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.environ['CHRONO_DATA'], '..'))

# Initialize the system and visualization
sys = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))

# Create the CityBus vehicle
car = veh.CityBus()
car.SetContactMethod(chrono.ChContactMethod_NSC)
car.SetChassisCollisionType(chrono.ChCollisionModel.NarrowPhase)
car.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI)))
car.Initialize()

# Set up the RigidTerrain
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Set up the camera to follow the vehicle
camera = vis.GetCamera(0)
camera.SetLookAt(chrono.ChVector3d(0, 3, 6), chrono.ChVector3d(0, 0, 0))

# Implement the interactive driver system
driver = veh.CityBusDriver(car)
driver.SetSteeringFunction(chrono.ChFunction_Const(0))
driver.SetThrottleFunction(chrono.ChFunction_Const(0))
driver.SetBrakeFunction(chrono.ChFunction_Const(0))

# Simulation loop
while vis.Run():
    # Update the driver and vehicle dynamics
    driver.Update(0.01)
    car.Update(0.01, False)

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Wait for the correct frame rate
    chrono.ChTime.GetWallClock().Sleep(0.02)