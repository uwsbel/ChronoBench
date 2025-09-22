import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors


chrono.SetChronoDataPath('../data/')  
system = chrono.ChSystemNSC()


gator = veh.Gator()
chassis = gator.GetChassisBody()


for wheel in gator.GetWheels():
    wheel.GetWheelShape().SetVisualizationType(chrono.ChVisualizationType.VISUALIZATION_POINT_CLOUD)

for steering in gator.GetSteerings():
    steering.GetSteeringShape().SetVisualizationType(chrono.ChVisualizationType.VISUALIZATION_MESH)


system.Add(gator)


terrain = chrono.ChBodyEasyBox(50, 0.5, 50, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -0.25, 0))
terrain.SetBodyFixed(True)
terrain.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(terrain)


driver = veh.ChDriver()
driver.SetVehicle(gator)
driver.SetSteeringController(veh.ChGeneric_2DofDriver::STEERING_CONTROLLER_SIMPLE_PI)
driver.SetThrottleController(veh.ChGeneric_2DofDriver::THROTTLE_CONTROLLER_SIMPLE_PI)
driver.SetBrakingController(veh.ChGeneric_2DofDriver::BRAKING_CONTROLLER_SIMPLE_PI)
driver.Initialize()


sensor_manager = sensors.ChSensorManager(system)
sensor_manager.ScaleVisualization(0.1)


point_light = sensors.ChPointLight()
point_light.SetPos(chrono.ChVectorD(5, 5, 5))
sensor_manager.AddPointLight(point_light)


camera = sensors.ChCamera()
camera.SetPos(chrono.ChVectorD(0, 1.5, -3))
camera.SetAttachToBody(chassis)
sensor_manager.AddCamera(camera)


visualization = chronoirr.ChVisualSystemIrrlicht()
visualization.AttachSystem(system)
visualization.SetWindowSize(800, 600)
visualization.SetWindowTitle('Gator Vehicle Simulation')
visualization.Initialize()
visualization.AddLogo()
visualization.AddSkyBox()
visualization.AddLightsWithShadows(sensor_manager.GetPointLights(), sensor_manager.GetDirectionalLights())


step_size = 0.01
total_time = 10.0

while (visualization.Run()):
    time = system.GetChronoTime()

    if (time < total_time):
        driver.Synchronize(time)
        system.DoStepDynamics(step_size)
        visualization.Update()
        sensor_manager.Update()

        
        camera.Render()
    else:
        break

visualization.Close()