import pychrono as chrono
import pychrono.irrlicht as irr


chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')
my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vehicle = chrono.CityBus()
vehicle.Initialize(my_system, 'vehicle')
vehicle.SetPos(chrono.ChVectorD(0, 0.5, 0))
vehicle.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))


vehicle.SetTireType(chrono.TireModelType_TMEASY)


terrain = chrono.ChBoxShape(20, 1, 20)
terrain_body = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True)
terrain_body.SetPos(chrono.ChVectorD(0, -0.5, 0))
my_system.Add(terrain_body)


app = irr.ChIrrApp(my_system, 'CityBus on Rigid Terrain', irr.dimension2du(1024, 768))
app.AddTypicalLogo()
app.AddTypicalSky()
app.AddTypicalCamera(irr.vector3df(0, 3, -6))
app.AddTypicalLights()
app.AddShadowAll()


driver = chrono.ChDriver(vehicle)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()
    app.DoStep()
    driver.Synchronize(my_system)
    my_system.DoStepDynamics(0.02)
    app.EndScene()