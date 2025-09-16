import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears', chronoirr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, 1.5))


truss = chrono.ChBodyEasyBox(2.0, 0.2, 2.0, 1000, True, True)
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(truss)
application.AddBody(truss)


bar_radius = 0.05
bar_length = 1.0
bar = chrono.ChBodyEasyCylinder(bar_radius, bar_length, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))  
bar.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.2, 0.2, 0.8))
system.Add(bar)
application.AddBody(bar)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(rev_joint)


rev_motor = rev_joint.GetMotor()
rev_motor.SetVelocityTarget(chrono.CH_C_2_PI)  
rev_motor.SetTorqueMax(1e6)


gear1_radius = 0.2
gear1_length = 0.1
gear1 = chrono.ChBodyEasyCylinder(gear1_radius, gear1_length, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 0))  
gear1.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.8, 0, 0))
system.Add(gear1)
application.AddBody(gear1)


gear1_joint = chrono.ChLinkLockRevolute()
gear1_joint.Initialize(bar, gear1, chrono.ChFrameD(gear1.GetPos(), chrono.QUNIT))
system.AddLink(gear1_joint)


gear2_radius = 0.4
gear2_length = 0.1
gear2_pos_x = 0.5 + (gear1_radius + gear2_radius)  
gear2 = chrono.ChBodyEasyCylinder(gear2_radius, gear2_length, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(gear2_pos_x, 0, 0))
gear2.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0, 0.8, 0))
system.Add(gear2)
application.AddBody(gear2)


gear2_joint = chrono.ChLinkRevolute()
gear2_joint.Initialize(truss, gear2, chrono.ChFrameD(gear2.GetPos(), chrono.QUNIT))
system.AddLink(gear2_joint)


gear_ratio = - (gear1_radius / gear2_radius)  
gear_link = chrono.ChLinkGear()
gear_link.Initialize(gear1, gear2, chrono.ChFrameD())  
gear_link.SetGearRatio(gear_ratio)
system.Add(gear_link)


application.SetTimestep(0.005)
application.SetVideoframeSaveIntervalLS(-1)  

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, 'Epicyclic Gears', chronoirr.dimension2du(800, 600))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1.5, 1.5, 1.5))


truss = chrono.ChBodyEasyBox(2.0, 0.2, 2.0, 1000, True, True)
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.5, 0.5, 0.5))
system.Add(truss)
application.AddBody(truss)


bar_radius = 0.05
bar_length = 1.0
bar = chrono.ChBodyEasyCylinder(bar_radius, bar_length, 1000, True, True)
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 0, 1)))  
bar.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.2, 0.2, 0.8))
system.Add(bar)
application.AddBody(bar)


rev_joint = chrono.ChLinkRevolute()
rev_joint.Initialize(truss, bar, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
system.AddLink(rev_joint)


rev_motor = rev_joint.GetMotor()
rev_motor.SetVelocityTarget(chrono.CH_C_2_PI)  
rev_motor.SetTorqueMax(1e6)


gear1_radius = 0.2
gear1_length = 0.1
gear1 = chrono.ChBodyEasyCylinder(gear1_radius, gear1_length, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.5, 0, 0))  
gear1.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0.8, 0, 0))
system.Add(gear1)
application.AddBody(gear1)


gear1_joint = chrono.ChLinkLockRevolute()
gear1_joint.Initialize(bar, gear1, chrono.ChFrameD(gear1.GetPos(), chrono.QUNIT))
system.AddLink(gear1_joint)


gear2_radius = 0.4
gear2_length = 0.1
gear2_pos_x = 0.5 + (gear1_radius + gear2_radius)  
gear2 = chrono.ChBodyEasyCylinder(gear2_radius, gear2_length, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(gear2_pos_x, 0, 0))
gear2.GetVisualShape(0).SetMaterialColor(chrono.ChColor(0, 0.8, 0))
system.Add(gear2)
application.AddBody(gear2)


gear2_joint = chrono.ChLinkRevolute()
gear2_joint.Initialize(truss, gear2, chrono.ChFrameD(gear2.GetPos(), chrono.QUNIT))
system.AddLink(gear2_joint)


gear_ratio = - (gear1_radius / gear2_radius)  
gear_link = chrono.ChLinkGear()
gear_link.Initialize(gear1, gear2, chrono.ChFrameD())  
gear_link.SetGearRatio(gear_ratio)
system.Add(gear_link)


application.SetTimestep(0.005)
application.SetVideoframeSaveIntervalLS(-1)  

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()