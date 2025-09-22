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
vehicle.SetPos(chrono.ChVectorD(0, 0, 1))
vehicle.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
vehicle.SetTirePressure(22.5)
vehicle.SetTireRadius(0.5)
vehicle.SetTireWidth(0.3)
vehicle.SetTireStep(0.01)
vehicle.SetTireFriction(3.0)
vehicle.SetTireRestitution(0.2)
vehicle.SetTireDampingFreq(1.0)
vehicle.SetTireDampingRelax(1.0)
vehicle.SetTireForceAppPoint(chrono.ChVectorD(0, 0, 0))
vehicle.SetTireTorqueAppPoint(chrono.ChVectorD(0, 0, 0))
vehicle.SetTireRollingResistance(0.01)
vehicle.SetTireRollingResistance_coef(0.01)
vehicle.SetTireRollingResistance_exp(1.0)
vehicle.SetTireRollingResistance_offset(0.0)
vehicle.SetTireRollingResistance_scale(1.0)
vehicle.SetTireRollingResistance_curvature(0.0)
vehicle.SetTireRollingResistance_curvature_coef(0.0)
vehicle.SetTireRollingResistance_curvature_exp(1.0)
vehicle.SetTireRollingResistance_curvature_offset(0.0)
vehicle.SetTireRollingResistance_curvature_scale(1.0)
vehicle.SetTireRollingResistance_curvature_curvature(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_coef(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_exp(1.0)
vehicle.SetTireRollingResistance_curvature_curvature_offset(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_scale(1.0)
vehicle.SetTireRollingResistance_curvature_curvature_curvature(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_curvature_coef(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_curvature_exp(1.0)
vehicle.SetTireRollingResistance_curvature_curvature_curvature_offset(0.0)
vehicle.SetTireRollingResistance_curvature_curvature_curvature_scale(1.0)
physics_engine.Add(vehicle)


driver = chrono.ChIrrGuiDriver()
driver.SetSleepInterval(1.0 / 50.0)
driver.Initialize()
driver.AddTypicalSky()
driver.AddTypicalLights()
driver.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
driver.AddTypicalCamera(chrono.ChVectorD(0, 0, 2))
driver.SetChaseCamera(chrono.ChVectorD(0, 0, 2), 10, 0.05)
driver.InitializeVisualization(physics_engine)


while driver.Run():
    physics_engine.DoStepDynamics(driver.GetSleepInterval())
    driver.BeginScene()
    driver.DrawAll()
    driver.EndScene()


driver.Close()
physics_engine.Clear()