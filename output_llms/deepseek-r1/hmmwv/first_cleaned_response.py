import os
import math
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))






init_loc = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.Initialize(veh.INITPOS, init_loc, init_rot)





terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChVectorD(0, 0, 0), 
                        chrono.ChVectorD(0, 0, 1), 
                        100, 100)
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()





vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


hmmwv.AttachVisualSystem(vis)





driver = veh.ChInteractiveDriverIRR(vis)


steering_time = 1.0
driver.SetSteeringDelta(0.02 / steering_time)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.SetGains(2.0, 5.0, 5.0)

hmmwv.GetVehicle().SetDriver(driver)






step_size = 0.02
frame_rate = 50
real_time_step = 1.0 / frame_rate
time = 0.0

while vis.Run():
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    
    
    time = system.GetChTime()
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    chrono.ChRealtimeStepTimer.Spin(real_time_step)