import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np


chrono.SetChronoOutputMode(chrono.ChronoLogMode.CHRONO_LOG_CONSOLE)
chrono.SetVerboseMode(True)


vehicle_system = chrono.ChSystemNSC()
vehicle_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


gator = veh.Gator(vehicle_system)
gator.Initialize(chrono.ChCoordinateD(0, 0, 0.5), 0.0)
gator.SetChassisVisualizationType(veh.VisualizationType.MESH)
gator.SetChassisFixed(False)
gator.SetTireType(veh.TireModelType.TMEASY)
gator.SetTireStepSize(1e-3)


for axle in gator.GetAxles():
    for wheel in axle.GetWheels():
        wheel.SetVisualizationType(veh.VisualizationType.PRIMITIVES)


terrain = veh.RigidTerrain(vehicle_system)
terrain.SetContactFrictionCoefficient(0.9)
terrain.SetContactRestitutionCoefficient(0.01)
terrain.SetContactMaterialProperties(2e7, 0.3)
terrain.Initialize(0, 0, 0)


driver = veh.ChInteractiveDriverIRR(gator)
driver.SetSteeringDelta(0.01)
driver.SetThrottleDelta(0.01)
driver.SetBrakingDelta(0.01)
driver.Initialize()


sensor_manager = sens.ChSensorManager(vehicle_system)


camera = sens.ChCameraSensor(
    gator.GetChassisBody(),  
    10,                      
    chrono.ChFrameD(chrono.ChVectorD(1.5, 0, 1.5), chrono.Q_from_AngAxis(-chrono.CH_PI_2, chrono.ChVectorD(1, 0, 0))),
    640,                     
    480,                     
    1.02                     
)
sensor_manager.AddSensor(camera)


light1 = sens.ChPointLight(gator.GetChassisBody(), chrono.ChVectorD(0.5, 0.5, 2), chrono.ChColor(1, 1, 1))
light2 = sens.ChPointLight(gator.GetChassisBody(), chrono.ChVectorD(-0.5, -0.5, 2), chrono.ChColor(1, 1, 1))
sensor_manager.AddSensor(light1)
sensor_manager.AddSensor(light2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(vehicle_system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Gator Vehicle Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(5, 0, 2), chrono.ChVectorD(0, 0, 0.5))
vis.SetCameraMovementType(chronoirr.CameraMovementType.TRACKBALL)
vis.AttachVehicle(gator.GetChassisBody())


time_step = 0.01
max_time = 30.0

while vis.Run():
    time = vehicle_system.GetChTime()

    if time >= max_time:
        break

    
    driver.Synchronize(time)
    driver.Advance(time_step)

    
    terrain.Synchronize(time)

    
    gator.Synchronize(time, driver.GetSteering(), driver.GetThrottle(), driver.GetBraking())
    gator.Advance(time_step)

    
    sensor_manager.Update()

    
    camera_data = camera.GetMostRecentData()
    if camera_data:
        
        pass

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    vehicle_system.DoStepDynamics(time_step)

    
    print(f"Simulation time: {time:.2f} s")


sensor_manager.RemoveAllSensors()
vis.RemoveAllObjects()