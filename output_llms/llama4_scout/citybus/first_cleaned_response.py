import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh


chrono.ChEngine.Initialize()


SIMULATION_FPS = 50
TIME_STEP = 1.0 / SIMULATION_FPS


app = irr.IrrlichtApplication()
app.SetSimulationFPS(SIMULATION_FPS)


simulation = chrono.ChSimulation()
simulation.SetTimestep(TIME_STEP)


citybus = veh.CityBus(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 1), chrono.ChQuaternionD(1, 0, 0, 0)))


citybus.SetChassisVisualizationType(veh.ChassisVisualizationType.MESH)
citybus.SetWheelVisualizationType(veh.WheelVisualizationType.PRIMITIVE)
citybus.SetTireModel(veh.TireModel.TME)


citybus.Initialize(simulation.GetPhysicsSystem())


citybus.SetPos(chrono.ChVectorD(0, 0, 1))
citybus.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


simulation.Add(citybus.GetChassisBody())


terrain = chrono.ChTerrain()
terrain.SetGridSize(100, 100)
terrain.SetLocation(chrono.ChVectorD(0, 0, -0.5))
terrain.SetTexture(chrono.ChTexture("textures/terrain.jpg"))


simulation.Add(terrain.GetGroundBody())


driver = veh.Driver()
driver.Initialize(citybus)


driver.SetSteering(chrono.ChFunction_Constant(0))
driver.SetThrottle(chrono.ChFunction_Constant(0.5))
driver.SetBraking(chrono.ChFunction_Constant(0))


simulation.Add(driver)


scene_node = app.AddChronoSimulation(simulation, "CityBus Simulation")


camera = scene_node.GetSceneManager().getActiveCamera()
camera.setPosition(irr.core.vector3df(10, 10, 10))
camera.setTarget(irr.core.vector3df(0, 0, 0))


def FollowVehicle():
    pos = citybus.GetChassisBody().GetPos()
    camera.setPosition(irr.core.vector3df(pos.x() + 10, pos.y() + 10, pos.z() + 10))
    camera.setTarget(irr.core.vector3df(pos.x(), pos.y(), pos.z()))


while app.Run():
    simulation.DoStepDynamics(TIME_STEP)
    FollowVehicle()

app.Close()
chrono.ChEngine.Finalize()