import math
import numpy as np
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


chrono.SetChronoDataPath('chrono_data/')


system = chrono.ChSystemSMC()


hmmwv = veh.HMMWV()
hmmwv.SetContactMethod(veh.ContactMethod_SMC)  
hmmwv.SetChassisCollisionType(veh.ChassisCollisionType_MESH)
hmmwv.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.5), chrono.QUNIT))


terrain = veh.RigidTerrain(system)
soil_mat = chrono.ChMaterialSurfaceSMC()
soil_mat.SetFriction(0.5)
soil_mat.SetDampingF(0.3)
soil_mat.SetCompliance(0.001)


height_map = np.random.normal(0, 0.1, (100, 100))
terrain_patch = terrain.AddPatch(soil_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 100, 100, height_map)
terrain.Initialize()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Deformable Terrain')
vis.AddCamera(chrono.ChVector3d(10, 10, 10), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


driver = veh.Driver()
driver.Initialize(hmmwv.GetDriverInputs())


simulation_time = 0.0
step_size = 0.01
frame_rate = 60


while vis.Run():
    
    simulation_time += step_size
    
    
    hmmwv.Synchronize(driver.GetInputs(), simulation_time)
    terrain.Synchronize(simulation_time)
    
    
    driver_inputs = driver.GetInputs()
    driver_inputs.m_throttle = 0.5  
    driver_inputs.m_steering = 0.0  
    driver_inputs.m_braking = 0.0   
    
    
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if vis.KeyHit(chrono.CH_KEY_ESCAPE):
        break


vis.Close()