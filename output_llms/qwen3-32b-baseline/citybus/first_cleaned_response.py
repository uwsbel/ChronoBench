import pychrono as chrono
import pychrono.vehicle as vehicle
import pychrono.irrlicht as irr


system = vehicle.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)


bus = vehicle.CityBus()
bus.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0)))


bus.GetChassis().SetVisualizationType(vehicle.VisualizationType_MESH)
for axle in bus.GetAxles():
    axle.suspension.SetVisualizationType(vehicle.VisualizationType_PRIMITIVES)
    axle.wheel_left.SetVisualizationType(vehicle.VisualizationType_MESH)
    axle.wheel_right.SetVisualizationType(vehicle.VisualizationType_MESH)


terrain = vehicle.RigidTerrain(system)
terrain.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 100, 100)
terrain.AddPatch(chrono.ChCoordsysD(), 100, 100)
terrain.Initialize()


bus.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)), 
               vehicle.ChassisCollisionType_NONE)


application = irr.ChIrrApp(system, "CityBus Simulation", irr.dimension2du(800, 600))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(irr.vector3df(0, 2.5, -6), irr.vector3df(0, 0, 1.2))
application.SetTimestep(0.02)


application.SetCameraTrack(bus.GetChassis().GetBody(), irr.vector3df(5, 3, 2))


driver = vehicle.ChInteractiveDriverIRR(application)
driver.SetSteeringClamp(1.0)
driver.SetThrottleClamp(1.0)
driver.SetBrakingClamp(1.0)
driver.Initialize()


while application.GetDevice().run():
    driver.Update()
    bus.SetDriverInputs(driver.GetInputs())
    
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    
    system.DoStepDynamics(0.02)