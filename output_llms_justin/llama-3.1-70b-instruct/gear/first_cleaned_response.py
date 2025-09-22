import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.engine as chronoengine


system = chrono.ChSystemNSC()
system.SetCollisionMargin(0.001)


visualizer = chronoirr.ChVisualSystemIrrlicht(system)
visualizer.SetWindowSize(800, 600)
visualizer.SetWindowTitle("Epicyclic Gears Simulation")
visualizer.SetCameraDistance(2.5)
visualizer.SetCameraAzimuth(90)
visualizer.SetCameraElevation(30)


truss = chrono.ChBodyEasyBox(system, 1, 0.1, 0.1, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)


bar = chrono.ChBodyEasyBox(system, 2, 0.1, 0.1, 1000, True, True)
bar.SetPos(chrono.ChVectorD(1, 0, 0))
bar.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(bar)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(bar, system.GetGroundBody())
motor.SetSpeed(chrono.CH_C_PI)
system.Add(motor)


gear1 = chrono.ChBodyEasyCylinder(system, 0.5, 0.1, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(2, 0, 0))
gear1.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(gear1)

gear2 = chrono.ChBodyEasyCylinder(system, 0.5, 0.1, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(3, 0, 0))
gear2.SetRot(chrono.Q_from_AngX(chrono.CH_C_PI_2))
system.Add(gear2)


contact = chrono.ChContactSurfaceNSC()
contact.SetMaterial(chrono.ChMaterialSurfaceNSC())
contact.SetCompliance(0.01)
contact.SetFriction(0.5)
system.AddContactMaterial(contact)
system.AddContact(gear1, gear2, contact)


contact = chrono.ChContactSurfaceNSC()
contact.SetMaterial(chrono.ChMaterialSurfaceNSC())
contact.SetCompliance(0.01)
contact.SetFriction(0.5)
system.AddContactMaterial(contact)
system.AddContact(gear1, bar, contact)
system.AddContact(gear2, bar, contact)


visualizer.AddSkyBox()
visualizer.AddTypicalLights()
visualizer.AddCameraVerticalAll()
visualizer.SetShowAxes(True)
visualizer.SetShowContactForces(True)
visualizer.SetShowContactPoints(True)


visualizer.Run()
system.SetTimestep(0.01)
while visualizer.Run():
    system.DoStepDynamics(0.01)