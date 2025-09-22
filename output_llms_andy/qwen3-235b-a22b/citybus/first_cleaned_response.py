import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


application = irr.ChIrrApp(system, "CityBus Simulation", irr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddCamera(chrono.ChVectorD(0, 3, -5), chrono.ChVectorD(0, 0, 0))


vehicle = veh.ChCityBus()
vehicle.Initialize(system, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI, chrono.ChVectorD(0, 1, 0))))
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(1.0 / 50.0)
vehicle.Initialize()


terrain = veh.ChRigidTerrain(system)
patch = terrain.AddPatch(chrono.ChCoordsysD(), chrono.ChVectorD(200, 1, 200), "concrete")
patch.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
terrain.Initialize()


driver = veh.ChDriverIRR(application.GetDevice(), vehicle)
driver.Initialize()


chassis = vehicle.GetChassis()
chassis.AddAsset(chrono.ChBoxShape(chrono.ChVectorD(1.5, 0.3, 3.0), chrono.ChVectorD(0, 0.3, 0)))
chassis.AddAsset(chrono.ChColorAsset(1.0, 0.8, 0.4))  

wheel_mesh = chrono.ChTriangleMeshConnected()
wheel_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/wheel.obj"))
wheel_vis = chrono.ChTriangleMeshShape()
wheel_vis.SetMesh(wheel_mesh)
chassis.AddAsset(wheel_vis)  


step_size = 1.0 / 50.0
rt_timer = chrono.ChRealtimeStepTimer()
time = 0.0


while application.GetDevice().run():
    rt_timer.Spin(step_size)
    
    
    driver_inputs = driver.GetInputs()
    vehicle.Synchronize(time, driver_inputs)
    system.DoStepDynamics(step_size)
    
    
    chassis_pos = chassis.GetPos()
    camera = application.GetSceneManager().getActiveCamera()
    camera.setPosition(chassis_pos + chrono.ChVectorD(0, 5, -10))
    camera.setTarget(chassis_pos)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    time += step_size