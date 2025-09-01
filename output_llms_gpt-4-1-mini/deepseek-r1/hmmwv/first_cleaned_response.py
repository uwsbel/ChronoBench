import os
import math
import time
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
system.SetSolverMaxIterations(150)




terrain = veh.RigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), 
                        chrono.ChVectorD(200, 200, 1))
patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
patch.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()




hmmwv = veh.HMMWV_Full(system)
hmmwv.SetContactMethod(chrono.ChContactMethod_NSC)
hmmwv.SetChassisFixed(False)
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), 
                                        chrono.Q_from_AngY(0)))
hmmwv.SetTireType(veh.TireModelType_TMEASY)
hmmwv.SetTireStepSize(0.001)
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
hmmwv.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 3, 1.5))
vis.AddTypicalLights()




driver = veh.ChIrrGuiDriver(vis)


driver.SetSteeringDelta(0.06)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()




step_size = 0.002
render_step = 1.0 / 50  
realtime_timer = chrono.ChRealtimeStepTimer()


while vis.Run():
    time = system.GetChTime()
    
    
    driver_inputs = driver.GetInputs()
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    terrain.Synchronize(time)
    
    
    hmmwv.Advance(step_size)
    terrain.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    realtime_timer.Spin(step_size)

    
    vis.Synchronize(f'{hmmwv.GetVehicle().GetVehicleSpeed():.2f} m/s', driver_inputs)