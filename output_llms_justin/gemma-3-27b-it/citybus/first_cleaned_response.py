import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh
import math






system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  






terrain = chrono.ChRigidBody()
terrain.SetBodyFixed(True)
terrain.SetMass(0)
terrain.AddAsset(chrono.ChTriangleMeshConnected())
terrain.GetAsset().LoadFromFile("terrain.obj")  
terrain.SetCollide(True)
system.Add(terrain)


bus = veh.ChBus()
bus.SetBodyFixed(False)
bus.SetMass(12000)
bus.SetPos(chrono.ChVectorD(0, 1, 0))
bus.SetTransform(chrono.ChQuaternionD(0, 0, 0))
system.Add(bus)






bus.GetVehicleModel().SetTireModel(veh.ChTireModel_TMeasy)


bus.SetForwardSpeed(0)






driver = veh.ChDriver()
driver.SetSteering(0)
driver.SetThrottle(0)
driver.SetBraking(0)
bus.SetDriver(driver)






vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -10))
vis.AddTypicalLights()






time_step = 0.005
simulation_fps = 50
max_simulation_time = 60


simulation_time = 0
while vis.Run() and simulation_time < max_simulation_time:
    
    if simulation_time < 10:
        driver.SetThrottle(1)
    else:
        driver.SetThrottle(0)
        driver.SetBraking(0.5)

    
    system.DoStepDynamics(time_step)
    simulation_time += time_step

    
    camera_pos = bus.GetPos() + chrono.ChVectorD(0, 5, -10)
    vis.GetRenderDevice().viewerManager.GetCamera().SetPosition(camera_pos)
    vis.GetRenderDevice().viewerManager.GetCamera().LookAt(bus.GetPos(), chrono.ChVectorD(0, 1, 0))

    
    vis.DrawMultithreaded(True)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.GetRenderDevice().closeDevice()