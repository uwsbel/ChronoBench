import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh





system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  





terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.AddAsset(chrono.ChTriangleMeshConnected())
terrain.GetMesh().LoadFromFile("terrain.obj")  
terrain.SetCollide(True)
system.Add(terrain)


texture = chrono.ChTexture()
texture.SetFilename("terrain_texture.jpg")  
terrain.GetMesh().SetTexture(texture)





bus = veh.ChBus()


initial_position = chrono.ChVectorD(0, 1, 0)
initial_orientation = chrono.ChQuaternionD(1, 0, 0, 0)
bus.SetPos(initial_position)
bus.SetRot(initial_orientation)


system.Add(bus)





driver = veh.ChDriver()
driver.SetSteering(0.0)
driver.SetThrottle(0.0)
driver.SetBraking(0.0)
bus.SetDriver(driver)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))  
vis.AddTypicalLights()





time_step = 1.0 / 50.0  
simulation_time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetChTime() < 5:
        driver.SetThrottle(0.5)
    else:
        driver.SetThrottle(0)
        driver.SetBraking(0.2)

    
    system.DoStepDynamics(time_step)

    simulation_time += time_step

    
    camera_pos = bus.GetPos() + chrono.ChVectorD(0, 2, -5)
    vis.GetCamera(0).SetPos(camera_pos)
    vis.GetCamera(0).SetLookAt(bus.GetPos())

print("Simulation finished.")