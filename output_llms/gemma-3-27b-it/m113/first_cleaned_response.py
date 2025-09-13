import chrono
import chrono.irrlicht as chronoirr
import chrono.vehicle as veh


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


terrain = chrono.ChTerrain()
terrain.Create(system, "terrain", chrono.ChVectorD(0, -1, 0), chrono.ChVectorD(100, 0, 100), 25, 25)
terrain.SetFriction(0.8)
terrain.SetRestitution(0.1)
system.Add(terrain)


vehicle = veh.ChM113()
vehicle.SetVehicleModelPath("data/M113/")  
vehicle.Initialize(system)
vehicle.SetChassisHeight(0.7)
vehicle.SetEngineType(veh.ChM113.EngineType.GASOLINE)
vehicle.SetTrackOffset(0.0)
vehicle.SetTrackWidth(1.5)


vehicle.SetPos(chrono.ChVectorD(0, 1, 0))
vehicle.SetTransform(chrono.ChFrameD(chrono.ChVectorD(0, 1, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 0, 1))))



driver = veh.ChDriver()
driver.SetInput(veh.ChDriver.INPUT_TYPE.SPEED, 1.0)  
driver.SetInput(veh.ChDriver.INPUT_TYPE.STEERING, 0.0)  
driver.Initialize(system, vehicle)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('M113 Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 2, -5))
vis.AddTypicalLights()


timestep = 0.005
time = 0.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(timestep)
    driver.Synchronize(timestep)

    time += timestep

    if time > 10:
        driver.SetInput(veh.ChDriver.INPUT_TYPE.SPEED, 0.0)  
        
    
    camera_pos = vehicle.GetPos() + chrono.ChVectorD(0, 2, -5)
    vis.GetCamera(0).SetPos(camera_pos)
    vis.GetCamera(0).SetLookAt(vehicle.GetPos())