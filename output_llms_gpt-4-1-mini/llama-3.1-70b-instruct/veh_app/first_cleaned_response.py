import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('./data/')


mysystem = chrono.ChSystemNSC()
chrono.ChCollisionSystem.SetDefaultImpulseThreshold(1e-3)


gator = veh.Gator()
gator.SetContactMaterial(chrono.ChMaterialSurfaceNSC())
gator.SetChassisVisualizationType(chrono.ChVisualizationType.MESH)
gator.SetSuspensionVisualizationType(chrono.ChVisualizationType.MESH)
gator.SetSteeringVisualizationType(chrono.ChVisualizationType.MESH)
gator.SetWheelVisualizationType(chrono.ChVisualizationType.MESH)
gator.SetBodyVisualizationType(chrono.ChVisualizationType.MESH)


mysystem.Add(gator.GetChassisBody())
mysystem.Add(gator.GetChassisBodyAuxRef())


terrain = chrono.ChBodyEasyBox(mysystem, 10, 10, 1, 1000, True, True)
terrain.SetPos(chrono.ChVectorD(0, -3, 0))
terrain.SetMaterial(chrono.ChMaterialSurfaceNSC())
mysystem.Add(terrain)


driver = veh.ChIrrGuiDriver(gator)
driver.SetTrackMode(chrono.ChIrrGuiDriver.TRACK_MODE_ISO_ZOOM)
driver.SetCameraDistance(2.5)
driver.SetCameraElevation(1.5)
driver.SetCameraAzimuth(1.57)
driver.SetCameraLookingAt(chrono.ChVectorD(0, 1.5, 0))


sensor_manager = sens.ChSensorManager(mysystem)
sensor_manager.SetSensorSamplingStep(0.01)


point_light = sens.ChPointLight()
point_light.SetPosition(chrono.ChVectorD(0, 5, 0))
point_light.SetIntensity(10)
sensor_manager.AddSensor(point_light)


camera = sens.ChCamera()
camera.SetPosition(chrono.ChVectorD(0, 2, 0))
camera.SetLookAt(chrono.ChVectorD(0, 1.5, 0))
camera.SetUp(chrono.ChVectorD(0, 0, 1))
camera.SetFov(60)
camera.SetResolution(640, 480)
sensor_manager.AddSensor(camera)


tstep = 0.01
tend = 10

while mysystem.GetChTime() < tend:
    
    driver.Synchronize(tstep)
    driver.Advance(tstep)

    
    terrain.Update(tstep)

    
    gator.Synchronize(tstep)
    gator.Advance(tstep)

    
    sensor_manager.Synchronize(tstep)
    sensor_manager.Advance(tstep)

    
    camera.Render()

    
    mysystem.DoStepDynamics(tstep)


mysystem.GetCollisionSystem().ReportAllContacts()