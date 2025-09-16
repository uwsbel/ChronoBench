import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import os


chrono.SetChronoDataPath(os.path.dirname(chrono.__file__) + '/data/')
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))



city_bus = veh.CityBus()
city_bus.Initialize(my_system)
city_bus.SetChassisPos(chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0)))
city_bus.SetChassisFixed(False)
city_bus.SetTireType(veh.TireModelType_TMEASY)
city_bus.SetTireStepSize(0.05)


city_bus.SetChassisVisualizationType(veh.VisualizationType_MESH)
city_bus.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
city_bus.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
city_bus.SetWheelVisualizationType(veh.VisualizationType_MESH)


terrain = veh.RigidTerrain(city_bus.GetSystem())
terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"))
terrain.Initialize(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)), chrono.ChVectorD(0, 0, 0), 50, 50, 1, 1)


driver = veh.ChDriver(city_bus.GetVehicle())
driver.SetSteeringControllerType(veh.SteeringControllerType_PID)
driver.SetSpeedControllerType(veh.SpeedControllerType_PID)
driver.Initialize()



camera = irr.ChCamera(irr.ChVector3df(0, 3, -5))
camera.SetLookAtPoint(irr.ChVector3df(0, 0, 0))


app = irr.ChIrrApp(my_system, camera)
app.AddTypicalLights()
app.AddSkyBox()
app.AddLogo()
app.AddLightWithShadow(irr.ChVector3df(1.5, 5.5, -2.5), irr.ChVector3df(0, 0, 0), 3, 2, 10, 40, 512)
app.SetTimestep(0.02)
app.SetTryRealtime(True)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    app.EndScene()