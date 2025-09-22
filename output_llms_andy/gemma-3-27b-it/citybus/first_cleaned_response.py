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
bus.SetVehicleModel("data/vehicle/citybus") 
bus.SetInitialPosition(chrono.ChVectorD(0, 1, 0))
bus.SetInitialOrientation(chrono.ChQuaternionD(1, 0, 0, 0))
bus.SetTireModel(veh.ChTireModel_TMeasy) 
system.Add(bus)






terrain.SetMaterialTexture(chrono.ChTexture("data/terrain/texture.png")) 


bus.SetBodyFixed(False)
bus.SetCollide(True)
bus.SetVisualizationType(veh.ChVehicleVisualizationType.MESH)






driver = veh.ChDriver()
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.1)
driver.SetBrakingDelta(0.1)
bus.SetDriver(driver)





vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('CityBus Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()





time_step = 1.0 / 50.0  

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if vis.GetSystem().GetKeyEvent(chrono.ChEventKey.KEY_UP).GetPressed():
        driver.SetThrottle(1.0)
    elif vis.GetSystem().GetKeyEvent(chrono.ChEventKey.KEY_DOWN).GetPressed():
        driver.SetBraking(1.0)
    else:
        driver.SetThrottle(0.0)
        driver.SetBraking(0.0)

    if vis.GetSystem().GetKeyEvent(chrono.ChEventKey.KEY_LEFT).GetPressed():
        driver.SetSteering(-1.0)
    elif vis.GetSystem().GetKeyEvent(chrono.ChEventKey.KEY_RIGHT).GetPressed():
        driver.SetSteering(1.0)
    else:
        driver.SetSteering(0.0)

    system.DoStepDynamics(time_step)