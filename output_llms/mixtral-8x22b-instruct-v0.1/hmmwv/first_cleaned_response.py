import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


physics_engine = chrono.ChSystemNSC()
physics_engine.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))


terrain = chrono.ChBoxGround(chrono.ChVectorD(100, 100, 0), 0.1)
terrain.SetTexture(chrono.GetChronoDataFile('terrain/grass.jpg'), 200, 200)
physics_engine.Add(terrain)


vehicle = chrono.ChHMMWV()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetTireType(chrono.ChHMMWV::TMEASY)
vehicle.SetTireStep(0.01)
vehicle.SetInitPosition(chrono.ChVectorD(0, 0, 1.5))
vehicle.SetInitQuaternion(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetTireForce(chrono.ChVectorD(0, 0, 0))
vehicle.SetSteeringValue(0)
vehicle.SetBrakingValue(0)
vehicle.SetThrottleValue(0)


physics_engine.Add(vehicle)


driver = chrono.ChIrrGuiDriver(physics_engine)
driver.SetSleep(False)


driver.SetWindowTitle("HMMWV Simulation")
driver.SetWindowSize(800, 600)
driver.Initialize()
driver.AddTypicalSky()
driver.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddTypicalLights()
driver.AddTypicalCamera(chrono.ChVectorD(0, 0, 10))


vehicle.SetChassisColors(chrono.ChColor(0.5, 0.5, 0.5))
vehicle.SetWheelColor(chrono.ChColor(0.1, 0.1, 0.1))
vehicle.SetSteeringLinkColor(chrono.ChColor(0.8, 0.8, 0.8))
vehicle.SetSuspensionColor(chrono.ChColor(0.8, 0.8, 0.8))


while driver.Run():
    physics_engine.DoStepDynamics(0.02)
    driver.BeginScene()
    driver.DrawAll()
    driver.EndScene()


driver.Close()