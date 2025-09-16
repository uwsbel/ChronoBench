import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = veh.ChCityBus(system)
vehicle.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0))))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.Initialize()


terrain = veh.ChRigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(200, 1, 200), 'concrete')
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


application = irr.ChIrrApp(system, 'CityBus Simulation', irr.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -6), vehicle.GetChassis().GetPos())


driver = veh.ChIrrGuiDriver(application)
driver.SetSteeringDelta(0.05)
driver.SetThrottleDelta(0.05)
driver.SetBrakingDelta(0.05)


step_size = 1.0 / 50.0  
time = 0.0


while application.GetDevice().run():
    
    driver_inputs = driver.GetInputs()
    
    
    vehicle.Synchronize(driver_inputs, terrain)
    
    
    system.DoStepDynamics(step_size)
    time += step_size
    
    
    chassis_pos = vehicle.GetChassis().GetPos()
    camera_pos = chassis_pos + chrono.ChVectorD(0, 3, -6)
    application.GetSceneManager().getActiveCamera().setPosition(camera_pos)
    application.GetSceneManager().getActiveCamera().setTarget(chassis_pos)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()