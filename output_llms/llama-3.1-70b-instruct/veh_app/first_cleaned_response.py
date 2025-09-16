import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as chrono_vehicle


chrono.SetChronoDataPath('./data/')
system = chrono.ChSystemNSC()



gator = chrono_vehicle.ChGator()
gator.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
gator.SetChassisVisualization(chrono.ChVisualizationType.PRIMITIVES)
gator.SetSuspensionVisualization(chrono.ChVisualizationType.MESH)
gator.SetSteeringVisualization(chrono.ChVisualizationType.MESH)
gator.SetWheelVisualization(chrono.ChVisualizationType.MESH)
gator.SetChassisCollisionLevel(chrono.ChCollisionLevel.NO_COLLISION)
system.Add(gator)


terrain = chrono.ChTerrain()
terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, -0.8, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
terrain.SetVisualizationType(chrono.ChVisualizationType.MESH)
system.Add(terrain)


driver = chrono_vehicle.ChIrrlichtDriver()
system.Add(driver)


sensor_manager = chrono.ChSensorManager()
gator.AddSensorManager(sensor_manager)


light1 = chrono.ChLightPoint()
light1.SetPosition(chrono.ChVectorD(10, 10, 10))
light1.SetColor(chrono.ChColor(1, 1, 1))
sensor_manager.AddSensor(light1)

light2 = chrono.ChLightPoint()
light2.SetPosition(chrono.ChVectorD(-10, 10, 10))
light2.SetColor(chrono.ChColor(1, 1, 1))
sensor_manager.AddSensor(light2)

camera = chrono.ChCamera()
camera.SetPosition(chrono.ChVectorD(0, 1.5, -2))
camera.SetCOI(chrono.ChVectorD(0, 1, 0))
camera.SetFocalLength(50)
camera.SetSensorSize(chrono.ChVectorD(1.6, 0.9))
camera.SetImageSize(chrono.ChVectorI(1024, 768))
sensor_manager.AddSensor(camera)


step_size = 0.01
time_end = 10
while system.GetChTime() < time_end:
    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    gator.Synchronize(system.GetChTime())
    sensor_manager.Synchronize(system.GetChTime())
    system.DoStepDynamics(step_size)
    system.GetCollisionSystem().Update()
    system.GetCollisionSystem().ReportAllContacts()
    system.GetCollisionSystem().ReportAllContacts()


irrlicht = chronoirr.ChIrrlichtApplication(system, 'Gator Simulation', chronoirr dimension=[1024, 768])
irrlicht.AddTypicalLights()
irrlicht.AddTypicalSky()
irrlicht.AddTypicalLogo('logo_pychrono_alpha.png')
irrlicht.AddCamera(camera)
irrlicht.SetCameraDistance(0.5)
irrlicht.SetCameraAzimuth(30)
irrlicht.SetCameraInclination(30)
irrlicht.SetCameraUpDirection(chrono.ChVectorD(0, 0, 1))
irrlicht.SetTimestep(step_size)
irrlicht.SetTryRealtime(True)

while (irrlicht.Run()):
    driver.Synchronize(system.GetChTime())
    terrain.Synchronize(system.GetChTime())
    gator.Synchronize(system.GetChTime())
    sensor_manager.Synchronize(system.GetChTime())
    system.DoStepDynamics(step_size)
    system.GetCollisionSystem().Update()
    system.GetCollisionSystem().ReportAllContacts()
    system.GetCollisionSystem().ReportAllContacts()
    irrlicht.BeginScene()
    irrlicht.DrawAll()
    irrlicht.DoStep()
    irrlicht.EndScene()