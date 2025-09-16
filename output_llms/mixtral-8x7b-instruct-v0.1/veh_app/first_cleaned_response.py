import chrono
import chrono.collision as col
import chrono.sensor as sens
import chrono.vehicle as veh
import chrono.utility as uti


chrono.SetChronoDataPath('path/to/data/folder')
my_system = chrono.ChSystemSolverSH()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = veh.RigidTerrain(my_system, 'terrain', chrono.ChVectorD(0, 0, 0))
vehicle = veh.ChVehicleModelGator(my_system, 'gator')
driver = veh.ChDriverScripted(vehicle, 'driver')
sensor_manager = sens.ChSensorManager(my_system)


terrain.SetContactMethod(col.CMETHOD_DISCRETE)
terrain.SetContactBreakingThreshold(10000)


vehicle.GetChassis().SetName('chassis')
vehicle.GetChassis().SetCollide(False)
vehicle.GetChassis().SetBodyFixed(True)
vehicle.GetChassis().SetPos(chrono.ChVectorD(0, 0, 0))
vehicle.GetChassis().SetColor(chrono.ChColor(0.5, 0.5, 0.5))

vehicle.GetSuspension(0).SetVisualizationType(veh.VisualizationType.VT_SPHERE)
vehicle.GetSuspension(1).SetVisualizationType(veh.VisualizationType.VT_SPHERE)
vehicle.GetWheel(0).SetVisualizationType(veh.VisualizationType.VT_BOX)
vehicle.GetWheel(1).SetVisualizationType(veh.VisualizationType.VT_BOX)


terrain_shape = col.ChBoxShape()
terrain_shape.GetBoxGeometry().Size = chrono.ChVectorD(100, 100, 1)
terrain.AddShape(terrain_shape)


driver.SetScriptFunction(lambda t: chrono.ChVectorD(10 * chrono.cos(t), 0, 10 * chrono.sin(t)))


camera = sens.ChCameraSensor()
camera.SetPos(chrono.ChVectorD(0, 10, 15))
camera.SetFocus(chrono.ChVectorD(0, 0, 0))
camera.SetUp(chrono.ChVectorD(0, 1, 0))
sensor_manager.AddSensor(camera)

point_light = sens.ChPointLight()
point_light.SetPos(chrono.ChVectorD(0, 10, 15))
point_light.SetColor(chrono.ChColor(1, 1, 1))
sensor_manager.AddSensor(point_light)


my_system.SetSolverType(chrono.ChSolver.Type.CS_CONTACT_FORCE)
my_system.SetSolverMaxIterations(100)
my_system.SetSolverMinIterations(10)
my_system.SetSolverAccuracy(1e-6)
my_system.SetSolverCollisionMargin(0.01)

for i in range(1000):
    my_system.DoStepDynamics(1.0 / 60.0)
    sensor_manager.Update(my_system.GetChTime())


my_system.DoFrameDynamics()
my_system.DoSimulationAdvance()


camera.SaveImages('camera_images', 1000)