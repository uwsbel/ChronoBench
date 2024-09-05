import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr


chrono.ChSystemSMC()


vehicle = veh.Gator()


terrain = chrono.ChBodyEasyBox(200, 1, 200, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -1, 0))
terrain.SetBodyFixed(True)


driver = veh.ChInteractiveDriver(vehicle)


sensor_manager = chrono.ChSensorManager()


point_light = chrono.ChPointLight(chrono.ChColor(1, 1, 1), 100, 1000)
point_light.SetPos(chrono.ChVectorD(0, 10, 0))
sensor_manager.AddLight(point_light)


camera = chrono.ChCamera(vehicle.GetChassisBody())
camera.SetPosition(chrono.ChVectorD(0, 1.5, -2))
camera.SetAim(chrono.ChVectorD(0, 0, 1))
sensor_manager.AddSensor(camera)


vehicle.GetChassisVisualization().SetDrawMode(chronoirr.EMD_WIREFRAME)
vehicle.GetWheel(0).GetVisualization().SetDrawMode(chronoirr.EMD_SOLID)
vehicle.GetWheel(1).GetVisualization().SetDrawMode(chronoirr.EMD_SOLID)
vehicle.GetWheel(2).GetVisualization().SetDrawMode(chronoirr.EMD_SOLID)
vehicle.GetWheel(3).GetVisualization().SetDrawMode(chronoirr.EMD_SOLID)


driver.SetSteeringMode(veh.ChInteractiveDriver.SteeringMode_KEYBOARD)
driver.SetThrottleMode(veh.ChInteractiveDriver.ThrottleMode_KEYBOARD)
driver.SetBrakingMode(veh.ChInteractiveDriver.BrakingMode_KEYBOARD)


sensor_manager.SetName("Sensor Manager")
sensor_manager.SetVerbose(True)


application = chronoirr.ChIrrApp(vehicle.GetSystem(), "Gator Simulation", chronoirr.dimension2du(1024, 768))


application.AddTypicalLights()
application.AddLight(point_light)
application.AssetBindAll()
application.AssetUpdateAll()
application.AddCamera(camera)
application.SetCamera(camera)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    driver.Synchronize(application.GetSystem())
    terrain.Synchronize(application.GetSystem())
    vehicle.Synchronize(application.GetSystem())
    sensor_manager.Synchronize(application.GetSystem())

    
    application.GetSystem().DoStepDynamics(0.01)

    
    sensor_manager.Render()

    application.EndScene()