import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set Chrono data paths
chrono.SetChronoDataPath(os.path.join(os.path.expanduser("~"), "PyChrono_data"))
veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

# Create the simulation system
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create rigid terrain
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                        veh.RigidTerrain.PatchType_BOX, 
                        200, 100)  # Size: length, width
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()

# Create MAN 10t truck vehicle
vehicle = veh.MAN_10t(sys)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0.5, 0), chrono.QUNIT))
vehicle.SetTireType(veh.TMEASY)  # Set TMEASY tire model
vehicle.SetTireStepSize(1e-3)    # Tire integration step size
vehicle.Initialize()

# Set up driver system for real-time controls
driver = veh.ChInteractiveDriverIRR(sys, vehicle, None, 20)
driver.SetSteeringDelta(0.05)    # Sensitivity for steering
driver.SetThrottleDelta(0.05)    # Sensitivity for throttle
driver.SetBrakingDelta(0.05)     # Sensitivity for braking

# Initialize Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('MAN 10t Truck Simulation - TMEASY Tires on Rigid Terrain')
vis.Initialize()

# Configure visualization assets
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, -6), vehicle.GetChassis().GetPos())  # Chase camera
vis.AddTypicalLights(chrono.ChVector3d(30, 100, -50),  # Position
                     chrono.ChColor(0.7, 0.7, 0.7),    # Ambient
                     chrono.ChColor(1.0, 1.0, 1.0),    # Diffuse
                     chrono.ChColor(0.8, 0.8, 0.8))    # Specular

# Simulation loop
while vis.Run():
    time = sys.GetChTime()
    
    # Driver inputs processing
    driver_inputs = driver.GetInputs()
    driver.Synchronize(time)
    
    # Synchronize vehicle and terrain
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    
    # Advance simulation
    sys.DoStepDynamics(1e-3)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Advance driver controls
    driver.Advance(1e-3)