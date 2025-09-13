import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math




STEP_SIZE = 0.005  
VISUALIZATION_FPS = 60  




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))  





init_pos = chrono.ChVectorD(0, 0, 0.5)
init_rot = chrono.ChQuaternionD(1, 0, 0, 0)


hmmwv = veh.HMMWV_Full(system)
hmmwv.SetInitPosition(chrono.ChCoordsysD(init_pos, init_rot))
hmmwv.SetEngineType(veh.EngineModelType_SHAFTS)  
hmmwv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)  
hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)  
hmmwv.SetTireType(veh.TireModelType_RIGID)  


hmmwv.Initialize()
hmmwv.SetChassisVisualizationType(veh.VisualizationType_MESH)  
hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetSteeringVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetWheelVisualizationType(veh.VisualizationType_MESH)
hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)


driver_data = veh.VehicleDriverData()
driver_data.m_delay = 0.4
vehicle = hmmwv.GetVehicle()




terrain = veh.RigidTerrain(system)


patch1 = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT),
                          chrono.ChVectorD(50, 50, 1))
patch1.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 50, 50)
patch1.SetColor(chrono.ChColor(0.8, 0.8, 0.8))


patch2 = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(30, 0, 0.1), chrono.QUNIT),
                          chrono.ChVectorD(20, 30, 1))
patch2.SetTexture(veh.GetDataFile("terrain/textures/dirt.jpg"), 20, 30)
patch2.SetColor(chrono.ChColor(0.5, 0.4, 0.3))


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/bump.obj"))
bump = chrono.ChBody()
bump.SetPos(chrono.ChVectorD(15, 0, 0))
bump.AddAsset(chrono.ChTriangleMeshShape(mesh))
bump.GetCollisionModel().ClearModel()
bump.GetCollisionModel().AddTriangleMesh(mesh, False, False)
bump.GetCollisionModel().BuildModel()
bump.SetCollide(True)
system.Add(bump)


heightmap = terrain.AddPatch(chrono.ChCoordsysD(chrono.ChVectorD(0, -20, 0), chrono.QUNIT),
                            veh.GetDataFile("terrain/height_maps/test64.bmp"), 
                            "test64", 64, 64, 0, 10)
heightmap.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 64, 64)

terrain.Initialize()




driver = veh.InteractiveDriverIRR(vehicle, "Terrain Test")
driver.SetSteeringDelta(0.02)
driver.SetThrottleDelta(0.02)
driver.SetBrakingDelta(0.06)
driver.Initialize()




vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('HMMWV on Complex Terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(10, -10, 2), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


terrain.AddVisualizationAssets(irr.VISUALIZATION_TYPE_PRIMITIVES, True)




realtime_timer = chrono.ChRealtimeStepTimer()
frame_interval = 1.0 / VISUALIZATION_FPS
next_frame_time = 0

while vis.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    
    
    hmmwv.Synchronize(time, driver.GetInputs(), terrain)
    terrain.Synchronize(time)
    
    
    system.DoStepDynamics(STEP_SIZE)
    
    
    if time >= next_frame_time:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        next_frame_time += frame_interval
    
    
    driver.Advance(STEP_SIZE)
    hmmwv.Advance(STEP_SIZE)
    vis.Synchronize(driver.GetInputModeAsString(), driver.GetInputs())
    
    realtime_timer.Spin(STEP_SIZE)