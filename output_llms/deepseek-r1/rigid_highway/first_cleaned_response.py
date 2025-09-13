import os
import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemSMC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.TerrainMesh(system)
terrain.SetMeshFilename(chrono.GetChronoDataFile("models/terrain/Highway_col.obj"))  
terrain.SetVisualMeshFilename(chrono.GetChronoDataFile("models/terrain/Highway_vis.obj"))  
terrain.SetContactFrictionCoefficient(0.8)  
terrain.Initialize()


hmmwv = veh.HMMWV_Full(system)
hmmwv.SetChassisFixed(False)  
hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.QUNIT))
hmmwv.SetTireType(veh.TireModelType_TMEASY)  
hmmwv.SetTireStepSize(0.001)  
hmmwv.Initialize()


hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


driver = veh.ChIrrGuiDriver(irr.irrlicht)  


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("HMMWV on Custom Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(6, 6, 1.5), chrono.ChVectorD(0, 0, 0.5))
vis.AddTypicalLights()


driver.SetInputMode(veh.InputMode_KEYBOARD)  
driver.Initialize()


step_size = 0.002
frame_interval = 0.02  
realtime_timer = chrono.ChRealtimeStepTimer()
frame_number = 0

while vis.Run():
    time = system.GetChTime()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    driver_inputs = driver.GetInputs()
    hmmwv.GetVehicle().Synchronize(time, driver_inputs, terrain)
    
    
    driver.Advance(step_size)
    system.DoStepDynamics(step_size)
    
    
    realtime_timer.Spin(step_size)
    
    
    if frame_number % int(frame_interval / step_size) == 0:
        vis.Render()
        frame_number = 0
    frame_number += 1