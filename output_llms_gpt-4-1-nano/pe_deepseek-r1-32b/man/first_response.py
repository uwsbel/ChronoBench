import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.robot as rob

# Set Chrono data path
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "chrono_data"))

# Create the vehicle system
sys = chrono.ChSystemSMC()
sys.SetGravity(chrono.ChVectorD(0, -9.81, 0))

# Create the vehicle
vehicle = veh.Vehicle(sys, veh.VehicleType.MAN_10t, True, True)

# Set vehicle properties
vehicle.SetContactMethod(veh.ContactMethodTypeEnum.TMEASY)  # TMEASY tire model
vehicle.SetChassisCollisionType(veh.ChassisCollisionTypeEnum.BOX)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.Initialize()

# Create the driver
driver = veh.Driver(vehicle.GetDriverInputs(), True)
driver.SetInputs(0, 0, 0)  # Steering, throttle, braking
vehicle.SetDriver(driver)

# Create the terrain
terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurface()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.1)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("MAN 10t Truck Simulation")
vis.SetAntiAliasing(True)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.AddTypicalLights()

# Chase camera setup
chase_cam = chronoirr.ChIrrChaseCamera()
chase_cam.SetDistance(10)
chase_cam.SetRotation(chrono.ChQuaternionD(chrono.Q_from_AngX(math.radians(45))))
vis.SetCamera(chase_cam.GetCamera())

# Add terrain texture
terrain_texture = chronoirr.ChIrrTexture()
terrain_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/ground.jpg"))
terrain_texture.SetRepeatX(10)
terrain_texture.SetRepeatY(10)
terrain.GetTerrain().AddVisualShape(terrain_texture)

# Simulation loop
while vis.Run():
    # Update driver inputs
    driver.SetInputs(steering, throttle, braking)
    
    # Update vehicle
    vehicle.Update()
    
    # Advance simulation
    sys.DoStepDynamics(0.01)
    
    # Render scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Small sleep to control frame rate
    chrono.ChSleep(0.01)