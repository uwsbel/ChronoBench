import pychrono as chrono
from pychrono import irrlicht as chronoirr
import numpy as np


system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, -9.81))
system.SetIterationNumber(100)
system.SetSolverType(chrono.ChSolver::Type_e_pcg)
system.SetTolerance(1e-10)


terrain = chrono.ChRigidTerrain(system)
terrain.GetGroundBody().SetPos(chrono.ChVectorD(0, 0, 0))
terrain.GetGroundBody().SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
terrain.SetTexture(chrono.ChTexture('assets/terrain_texture.jpg'))
terrain.SetColor(chrono.ChColor(0.5, 0.5, 0.5))
terrain.Initialize()



chassis = chrono.ChBodyEasyBox(2, 1, 0.5, 1000, True, True)
chassis.SetPos(chrono.ChVectorD(0, 0.5, 0))
chassis.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(chassis)


wheel_fl = chrono.ChBodyEasyCylinder(0.3, 0.2, 1000, True, True)
wheel_fl.SetPos(chrono.ChVectorD(-1, 0.25, 0.5))
wheel_fl.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(wheel_fl)

tire_fl = chrono.ChTireTMEASY()
tire_fl.SetRadius(0.3)
tire_fl.SetWidth(0.2)
tire_fl.SetStiffness(1000)
tire_fl.SetDamping(10)
tire_fl.SetFriction(1.0)
tire_fl.SetRollingResistance(0.01)
wheel_fl.AddTire(tire_fl)


wheel_fr = chrono.ChBodyEasyCylinder(0.3, 0.2, 1000, True, True)
wheel_fr.SetPos(chrono.ChVectorD(1, 0.25, 0.5))
wheel_fr.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(wheel_fr)
tire_fr = chrono.ChTireTMEASY()
tire_fr.SetRadius(0.3)
tire_fr.SetWidth(0.2)
tire_fr.SetStiffness(1000)
tire_fr.SetDamping(10)
tire_fr.SetFriction(1.0)
tire_fr.SetRollingResistance(0.01)
wheel_fr.AddTire(tire_fr)


wheel_rl = chrono.ChBodyEasyCylinder(0.3, 0.2, 1000, True, True)
wheel_rl.SetPos(chrono.ChVectorD(-1, 0.25, -0.5))
wheel_rl.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(wheel_rl)
tire_rl = chrono.ChTireTMEASY()
tire_rl.SetRadius(0.3)
tire_rl.SetWidth(0.2)
tire_rl.SetStiffness(1000)
tire_rl.SetDamping(10)
tire_rl.SetFriction(1.0)
tire_rl.SetRollingResistance(0.01)
wheel_rl.AddTire(tire_rl)


wheel_rr = chrono.ChBodyEasyCylinder(0.3, 0.2, 1000, True, True)
wheel_rr.SetPos(chrono.ChVectorD(1, 0.25, -0.5))
wheel_rr.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
system.Add(wheel_rr)
tire_rr = chrono.ChTireTMEASY()
tire_rr.SetRadius(0.3)
tire_rr.SetWidth(0.2)
tire_rr.SetStiffness(1000)
tire_rr.SetDamping(10)
tire_rr.SetFriction(1.0)
tire_rr.SetRollingResistance(0.01)
wheel_rr.AddTire(tire_rr)


spring_fl = chrono.ChLinkSpring()
spring_fl.SetSpringStiffness(10000)
spring_fl.SetDampingCoefficient(1000)
spring_fl.SetRestLength(0.5)
spring_fl.Initialize(chassis, wheel_fl, chrono.ChVectorD(0, 0, 0))
system.Add(spring_fl)

spring_fr = chrono.ChLinkSpring()
spring_fr.SetSpringStiffness(10000)
spring_fr.SetDampingCoefficient(1000)
spring_fr.SetRestLength(0.5)
spring_fr.Initialize(chassis, wheel_fr, chrono.ChVectorD(0, 0, 0))
system.Add(spring_fr)

spring_rl = chrono.ChLinkSpring()
spring_rl.SetSpringStiffness(10000)
spring_rl.SetDampingCoefficient(1000)
spring_rl.SetRestLength(0.5)
spring_rl.Initialize(chassis, wheel_rl, chrono.ChVectorD(0, 0, 0))
system.Add(spring_rl)

spring_rr = chrono.ChLinkSpring()
spring_rr.SetSpringStiffness(10000)
spring_rr.SetDampingCoefficient(1000)
spring_rr.SetRestLength(0.5)
spring_rr.Initialize(chassis, wheel_rr, chrono.ChVectorD(0, 0, 0))
system.Add(spring_rr)


driver = chrono.ChDriver()
driver.SetThrottle(0.5)
driver.SetSteering(0.0)
driver.SetBraking(0.0)


application = chronoirr.ChIrrApp(system, 'HMMWV Simulation', chronoirr.dimension2d(800, 600), chronoirr.vector3d(0, 0, -10))
application.AddLogo()
application.AddSkyBox(chrono.ChColor(0.5, 0.5, 0.5))
application.AddCamera(chrono.ChVectorD(0, 1, -5))
application.SetTimestep(1.0 / 50.0)


application.AddLightPoint(chrono.ChVectorD(0, 5, 5), 10)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()