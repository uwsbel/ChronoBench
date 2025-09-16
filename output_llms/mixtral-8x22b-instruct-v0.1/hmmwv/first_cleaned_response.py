import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


physics_engine = chrono.ChSystemNSC()
physics_engine.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = chrono.ChBoxGround(chrono.ChVectorD(100, 100, 0), 0.1)
terrain.SetContactMaterial(3e7, 0.3)
terrain.SetTexture(chrono.GetChronoDataFile('terrain/textures/concrete.jpg'))
physics_engine.Add(terrain)


vehicle = chrono.ChHMMWV()
vehicle.SetChassisFixed(False)
vehicle.SetFrame_COG_to_chassis(chrono.ChVectorD(0.0, 0.0, 0.75))
vehicle.SetMass(3300)
vehicle.SetInertiaXX(chrono.ChVectorD(500, 500, 500))
vehicle.SetContactMaterial(3e7, 0.4)
vehicle.SetTireType(chrono.ChHMMWV::TMEASY)
vehicle.SetTireStep(0.01)
vehicle.SetInitCondition(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0.75), chrono.QUNIT))
physics_engine.Add(vehicle)


driver = chrono.ChIrrGuiDriver()
driver.SetSleepInterval(1 / 50)
driver.SetTimestep(0.01)
driver.Initialize()


visualization = chronoirr.ChIrrApp(physics_engine, 'HMMWV Simulation', chronoirr.dimension2du(1024, 768))
visualization.AddTypicalSky()
visualization.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
visualization.AddTypicalCamera(chrono.ChVectorD(0, 0, 1.5))
visualization.AddLightWithShadow(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(0, 0, 0), 1, 5, 50, 512, 2048)


while visualization.Run():
    physics_engine.DoStepDynamics(driver.GetTimestep())
    visualization.BeginScene()
    visualization.DrawAll()
    visualization.EndScene()


visualization.Close()