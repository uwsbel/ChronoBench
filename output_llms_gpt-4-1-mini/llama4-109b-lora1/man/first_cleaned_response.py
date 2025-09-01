import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.vehicle.manifest as veh_man


chrono.SetChairsThreading(chrono.GetNumProcs())
chrono.SetChronoDataPath(veh.GetDataPath())
chrono.SetChronoUserDataPath(veh.GetDataPath())
chrono.SetChronoDriverDataPath(veh.GetDataPath())


vehicle = veh.GetMAN_10t(chrono.ChContactMethod_NSC)
terrain = veh.GetRigidTerrain(vehicle.GetSystem())
manager = veh.GetSystemManager(vehicle, terrain)


vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSuspensionCollisionType(veh.CollisionType_NONE)
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
vehicle.SetSteeringCollisionType(veh.CollisionType_NONE)
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
vehicle.SetWheelCollisionType(veh.CollisionType_NONE)


vehicle.Initialize(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1.1), chrono.ChQuaterniond(1, 0, 0, 0)))


application = irr.ChIrrApp(vehicle.GetSystem(), "MAN 10t Truck", 1280, 720)
application.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
application.AddSkyBox()
application.SetCameraVerticalFOV(chrono.DEG_TO_RAD * 50)
application.SetCameraDistance(6.0)
application.SetChaseCamera(vehicle.GetChassisBody(), chrono.ChVector3d(0.0, -4.0, 1.0))


application.AddDirectionalLight(chrono.ChVector3d(0.5, 0.5, 0.5), chrono.ChVector3d(-2, 5, 2), 0.8)
application.AddPointLight(chrono.ChVector3d(-2, 2, 2), chrono.ChVector3d(0.4, 0.4, 0.4), 0.8)
application.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(0.4, 0.4, 0.4), 0.8)
terrain.SetTexture(chrono.GetChronoDataPath() + "terrain/texture.jpg", 200, 200)
terrain.SetLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")


driver = veh.GetChVehicleDriver(vehicle)
driver.SetSteeringDelta(0.5 / 100)
driver.SetThrottleDelta(0.5 / 100)
driver.SetBrakingDelta(0.5 / 100)
driver.Initialize()


tire_vis = veh.GetChTMEASY_Tire(vehicle.GetSystem())
tire_collision = veh.GetChTMEASY_Tire(vehicle.GetSystem())
tire_collision.SetVisualizationType(veh.VisualizationType_NONE)
tire_collision.SetTireCollisionType(veh.CollisionType_TMEASY)
tire_collision.SetTireCollisionFamily(veh.CollisionFamily_TMEASY_STANDARD)
tire_collision.SetTireMass(3.0)
tire_collision.SetTireRadius(0.5)
tire_collision.SetTireWidth(0.2)
tire_collision.SetTireInflation(0.0)
tire_collision.SetTireCamber(chrono.DEG_TO_RAD * -2)


manager.Initialize()
manager.Synchronize(chrono.ChTimeStep())
manager.Advance(chrono.GetSimulationStep())


while not application.GetQuitRequestFlag():
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    manager.Synchronize(time)
    manager.Advance(chrono.GetSimulationStep())
    application.BeginScene()
    terrain.Render()
    vehicle.Render()
    driver.Render()
    application.EndScene()