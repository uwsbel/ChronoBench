import os
import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.robot as rob


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "chrono_data"))


sys = chrono.ChSystemSMC()
sys.SetGravity(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.Vehicle(sys, veh.VehicleType.MAN_10t, True, True)


vehicle.SetContactMethod(veh.ContactMethodTypeEnum.TMEASY)  
vehicle.SetChassisCollisionType(veh.ChassisCollisionTypeEnum.BOX)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))
vehicle.Initialize()


driver = veh.Driver(vehicle.GetDriverInputs(), True)
driver.SetInputs(0, 0, 0)  
vehicle.SetDriver(driver)


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChMaterialSurface()
patch_mat.SetFriction(0.8)
patch_mat.SetRestitution(0.1)
terrain.AddPatch(patch_mat, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT), 100, 100)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("MAN 10t Truck Simulation")
vis.SetAntiAliasing(True)
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, 10))
vis.AddTypicalLights()


chase_cam = chronoirr.ChIrrChaseCamera()
chase_cam.SetDistance(10)
chase_cam.SetRotation(chrono.ChQuaternionD(chrono.Q_from_AngX(math.radians(45))))
vis.SetCamera(chase_cam.GetCamera())


terrain_texture = chronoirr.ChIrrTexture()
terrain_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/ground.jpg"))
terrain_texture.SetRepeatX(10)
terrain_texture.SetRepeatY(10)
terrain.GetTerrain().AddVisualShape(terrain_texture)


while vis.Run():
    
    driver.SetInputs(steering, throttle, braking)
    
    
    vehicle.Update()
    
    
    sys.DoStepDynamics(0.01)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    chrono.ChSleep(0.01)