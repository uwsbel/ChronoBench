import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as robot
import pychrono.vehicle as veh
import pychrono.ros as chros
import pychrono.irrlicht as chronoirr


sys = chrono.ChSystemSMC()


contact_method = chrono.ChContactMethod_SMC
engine_type = veh.EngineModelType_SIMPLE_MAP
tire_model = veh.TireModelType_TMEASY
chassis_collision_type = veh.ChassisCollisionType_PRIMITIVES


hmmwv = veh.HMMWV_Full()
hmmwv.SetContactMethod(contact_method)
hmmwv.SetChassisCollisionType(chassis_collision_type)
hmmwv.SetEngineType(engine_type)
hmmwv.SetTireType(tire_model)
hmmwv.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
hmmwv.Initialize()


terrain = veh.RigidTerrain(sys)
patch_mat = chrono.ChContactMaterialSMC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 200, 200)
terrain.Initialize()


driver = veh.ChDriver(hmmwv.GetVehicle())


ros_manager = chros.ChROSMultiManager()
ros_manager.RegisterHandler(chros.ChROSClockHandler())  
ros_manager.RegisterHandler(chros.ChROSInputHandler(ros_manager, driver))  
ros_manager.RegisterHandler(chros.ChROSBodyStateHandler(ros_manager, hmmwv.GetChassisBody(), "hmmwv/state"))  
ros_manager.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('HMMWV Simulation with ROS')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 5, 10))
vis.AddTypicalLights()


time_step = 1e-3
sim_time = 0


while vis.Run():
    time = sys.GetChTime()
    
    
    driver.Synchronize(time)
    terrain.Synchronize(time)
    
    
    ros_manager.Update(time, time_step)
    
    
    sys.DoStepDynamics(time_step)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    sim_time += time_step