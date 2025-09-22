import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.set_default_G_acc(9.81)


system = ch.ChSystemEulerImplicit()


app = irr.IrrlichtApplication(system)


terrain = veh.RigidTerrain(system, 'data/terrain/granite.pcm')
terrain.SetPos(ch.ChVector3d(0, 0, 0))
terrain.SetCollisionMaterial(veh.RigidTerrain.CollisionMaterial.GRANITE)


vehicle = veh.MAN_10t(system)


vehicle.SetChassisVisualizationType(veh.ChassisVisualizationType.PRIMITIVES)
vehicle.SetChassisCollisionType(veh.ChassisCollisionType.PRIMITIVES)
vehicle.SetWheelVisualizationType(veh.WheelVisualizationType.SHAFTS)
vehicle.SetWheelCollisionType(veh.WheelCollisionType.SHAFTS)


tire = veh.TMEasyTire(system)
vehicle.SetTireModel(tire)


vehicle.SetPosition(ch.ChVector3d(0, 0, 1))
vehicle.SetHeading(0)


driver = veh.DriverWheeledVehicle(vehicle, system)
driver.SetSteeringDelta(0.1)
driver.SetThrottle(0.5)
driver.SetBraking(0.0)


vis = irr.IrrlichtVis(app, system)
vis.SetCameraDistance(10)
vis.SetCameraPos(ch.ChVector3d(0, -10, 5))
vis.SetChaseCamera(vehicle.GetChassisBody())


vis.AddDirectionalLight(ch.ChVector3d(5, 5, 5), ch.ChVector3d(-1, -1, -1), 1)
vis.AddSkyBox('data/textures/skybox')
vis.SetTerrainTexture('data/textures/granite.jpg')
vis.AddLogo('data/textures/logo.png')


app.SetSimulationRate(1.0 / 60.0)
app.Run()