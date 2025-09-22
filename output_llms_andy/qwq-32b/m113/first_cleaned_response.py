import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.CreateVehicle('M113')
vehicle.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)),  
                   veh.ChTireModelType.RIGID)
system.Add(vehicle.GetChassis())


terrain = veh.ChTerrain()
terrain.SetFriction(0.9)
terrain.SetRestitution(0.1)
terrain.SetHeight(0)  
terrain.SetSize(100, 100)  
terrain.Initialize()
system.Add(terrain)


driver = veh.ChDriver()
driver.SetThrottle(0.8)  
driver.SetSteering(0)    
vehicle.SetDriver(driver)


visualiz = irr.ChIrrApp(system, "M113 Simulation", irr.dimension2du(1024, 768))
visualiz.AddTypicalLights()
visualiz.AddTypicalCamera(chrono.ChVectorD(5, 5, 5))  
visualiz.AddAsset(chrono.ChColorAsset(0.8, 0.8, 0.8))  


vehicle.SetChassisVisualizationType(veh.ChVisualizationType::MESH)
vehicle.SetWheelVisualizationType(veh.ChVisualizationType::MESH)
visualiz.Add(vehicle.GetVisualization())
visualiz.Add(terrain.GetVisualization())


step_size = 0.01
realtime_timestep = step_size
realtime_sync = True
step_manager = chrono.ChRealtimeStepManager(step_size, realtime_timestep, realtime_sync)


visualiz.SetTimestep(step_size)
visualiz.SetVideoframeSave(True)
visualiz.SetVideoframeDeltaT(0.2)

while visualiz.Run():
    time = system.GetChTime()
    
    
    driver.Synchronize(time)
    vehicle.Synchronize(time)
    
    
    system.DoStepDynamics(step_size)
    
    
    visualiz Advance()
    
    
    step_manager.Synchronize()