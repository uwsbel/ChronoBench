import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.Irrlicht as irr




chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  




terrain = veh.RigidTerrain(system)
patch_mat = chrono.ChMaterialSurfaceNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, 
                         chrono.ChVectorD(0, 0, 0), 
                         chrono.ChVectorD(200, 200, 1))
patch.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
terrain.Initialize()




bus = veh.CityBus(system)
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1.2), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
bus.SetTireType(veh.TireModelType_TMeasy)
bus.SetTireStepSize(1e-3)
bus.Initialize()




application = irr.ChIrrApp(system, "CityBus Simulation", irr.dimension2du(1280, 720))
application.AddTypicalLights()
application.AddSkyBox()
application.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))


camera_pos = bus.GetChassisPos() + chrono.ChVectorD(0, 5, -10)
application.AddCamera(irr.vector3df(camera_pos.x, camera_pos.y, camera_pos.z),
                     irr.vector3df(bus.GetChassisPos().x, bus.GetChassisPos().y, bus.GetChassisPos().z))

application.AssetBindAll()
application.AssetUpdateAll()




driver = veh.ChIrrGuiDriver(application)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)




step_size = 0.02  
frame_rate = 50

while application.GetDevice().run():
    time = system.GetChTime()

    
    driver_inputs = driver.GetInputs()

    
    bus.Synchronize(time, driver_inputs)
    terrain.Synchronize(time)
    driver.Synchronize(time)
    application.Synchronize(time)

    
    system.DoStepDynamics(step_size)

    
    bus_pos = bus.GetChassisPos()
    cam_node = application.GetSceneManager().getActiveCamera()
    cam_node.setPosition(irr.vector3df(bus_pos.x, bus_pos.y + 5, bus_pos.z - 10))
    cam_node.setTarget(irr.vector3df(bus_pos.x, bus_pos.y, bus_pos.z))

    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()