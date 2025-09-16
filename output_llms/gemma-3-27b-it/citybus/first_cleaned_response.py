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


bus = veh.ChCityBus()
bus.SetVehicleModel("data/citybus")
bus.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
bus.SetChassisHeight(0.7)
bus.SetTireModel(veh.ChTireModel_TMeasy)
bus.SetEngineType(veh.ChEngineModel_Simple)


bus.SetPos(chrono.ChVectorD(0, 2, 0))
bus.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 2, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
system.Add(bus)


driver = veh.ChDriver()
driver.SetSteeringDelta(20)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.2)
bus.SetDriver(driver)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()


time_step = 1.0 / 50.0
simulation_time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetChTime() < 5:
        driver.SetSteering(0.5)
        driver.SetThrottle(1.0)
    else:
        driver.SetSteering(0.0)
        driver.SetThrottle(0.0)
        driver.SetBraking(1.0)

    system.DoStepDynamics(time_step)
    simulation_time += time_step