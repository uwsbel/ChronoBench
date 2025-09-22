import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('../../data/')
my_system = chrono.ChSystemNSC()


bmw_e90 = veh.BMW_E90(my_system)
bmw_e90.SetChassisVisualizationType(veh.VisualizationType_MESH)
bmw_e90.SetChassisCollisionType(veh.CollisionType_MESH)
bmw_e90.SetTireType(veh.TireModelType_TMEASY)


terrain = veh.RigidTerrain(my_system)
terrain.Initialize(veh.GetDataFile('terrain/Racetrack.obj'), veh.GetDataFile('terrain/textures/racetrack.jpg'), 1, 1, 1, 0)


driver = veh.ChDriver(bmw_e90)
driver.SetSteeringControllerType(veh.SteeringControllerType_PID)
driver.SetSpeedControllerType(veh.SpeedControllerType_PID)
driver.SetSteeringLookAheadDistance(5)
driver.SetMaxSpeed(10)


app = irr.ChIrrApp(bmw_e90.GetSystem(), 'BMW E90 on Racetrack', irr.dimension2du(1024, 768))
app.AddTypicalLights()
app.AddSkyBox()
app.AddCamera(irr.vector3df(0, 3, -5), irr.vector3df(0, 0, 0))
app.SetTimestep(0.01)


bmw_e90.SetPos(chrono.ChVectorD(0, 0.2, 0))
bmw_e90.SetChassisFixed(False)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()