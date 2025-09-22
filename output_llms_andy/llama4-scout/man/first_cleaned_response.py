import chrono as ch
import chrono.irrlicht as irr
import chrono.vehicle as veh


ch.set_default_G_acc(9.81)


system = ch.ChSystem()


app = irr.IrrlichtApplication(system)


terrain = veh.RigidTerrain(system)
terrain.SetGridSize(100, 100)
terrain.SetTextureSizes(1024, 1024)
terrain.AddLogo('chrono_logo.png', ch.Vector(0.5, 0.9, 0), ch.Vector(0.1, 0.1, 0))
terrain.AddSkyBox('skybox', ch.Vector(1, 1, 1))


vehicle = veh.MAN_10t(system)


vehicle.SetChassisVisualizationType(veh.VisualizationType.MESH)
vehicle.SetChassisCollisionType(veh.CollisionType.MESH)
vehicle.SetWheelVisualizationType(veh.VisualizationType.MESH)
vehicle.SetWheelCollisionType(veh.CollisionType.MESH)


tire_model = veh.TMEasyTire(system)
vehicle.ReplaceTire(tire_model)


driver = veh.RT_Driver(vehicle)
driver.SetThrottle(0.5)
driver.SetSteering(0.2)
driver.SetBraking(0.0)


system.Add(vehicle.GetChassisBody())
terrain.AddVehicle(vehicle)


camera = irr.ChaseCamera(vehicle.GetChassisBody(), ch.Vector(0, -10, 5), ch.Vector(0, 0, 0))


system.SetSolverType(ch.ChSolverType.PCG)
system.SetSolverIter(100)
system.SetTimestep(0.01)


app.SetTimestep(0.01)
app.SetShutdownRequestCallback(lambda: irr.ExitCode.USER_EXIT)
app.Run()