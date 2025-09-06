import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr

# Set the Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", 'data'))

# Create the physical system
sys = chrono.ChSystemSMC()

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('ARTcar Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Vehicle initialization parameters
contact_method = chrono.ChContactMethod_SMC
chassis_collision_type = veh.ChassisCollisionType_BOX
initLoc = chrono.ChVector3d(0, 0.5, 0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)  # No rotation

# Create and configure the ARTcar vehicle
car = veh.ARTcar()
car.SetContactMethod(contact_method)
car.SetChassisCollisionType(chassis_collision_type)
car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
car.Initialize(sys)  # Initialize the vehicle within the system

# Set up the rigid terrain with a custom texture
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200, 100)
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # Apply a concrete texture
terrain.Initialize()

# Create an interactive driver for vehicle control
driver = veh.ChInteractiveDriver(car)
driver.SetSteeringDelta(0.05)  # Sensitivity for steering
driver.SetThrottleDelta(0.1)   # Sensitivity for throttle
driver.SetBrakingDelta(0.1)    # Sensitivity for braking
driver.Initialize()

# Simulation loop parameters
time_step = 0.02  # 50 FPS (1/50 = 0.02)

# Run the simulation loop
while vis.Run():
    time = sys.GetChTime()
    
    # Synchronize components
    driver.Synchronize(time)
    terrain.Synchronize(time)
    car.Synchronize(time, driver.GetInputs(), terrain)
    
    # Advance the simulation
    sys.DoStepDynamics(time_step)
    
    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()