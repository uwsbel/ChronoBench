import pychrono as chrono
import pychrono.irrlicht as chronoirr


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


truss = chrono.ChBody()
truss.SetBodyFixed(True)
system.Add(truss)


bar = chrono.ChBody()
bar.SetPos(chrono.ChVectorD(0, 0, 0))
bar.SetMass(1.0)
bar.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
system.Add(bar)


rev_joint = chrono.ChLinkLockRevolute()
rev_joint.Initialize(truss, bar, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint)


gear1 = chrono.ChBody()
gear1.SetPos(chrono.ChVectorD(0.2, 0, 0))
gear1.SetMass(0.5)
gear1.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
system.Add(gear1)

gear2 = chrono.ChBody()
gear2.SetPos(chrono.ChVectorD(0.4, 0, 0))
gear2.SetMass(0.5)
gear2.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
system.Add(gear2)


rev_joint1 = chrono.ChLinkLockRevolute()
rev_joint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint1)

rev_joint2 = chrono.ChLinkLockRevolute()
rev_joint2.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(0.4, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.AddLink(rev_joint2)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)), chrono.ChFrameD(chrono.ChVectorD(0.4, 0, 0)))
gear_constraint.SetTransmissionRatio(-1)  
system.AddLink(gear_constraint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, gear1, chrono.ChFrameD(chrono.ChVectorD(0.2, 0, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  
system.AddLink(motor)


application = chronoirr.ChIrrApp(system, "Epicyclic Gear System", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, 1, 1))


visual_material = chrono.ChVisualMaterial()
visual_material.SetDiffuseColor(chrono.ChColor(0.8, 0.3, 0.3))

truss_shape = chrono.ChBoxShape()
truss_shape.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.1)
truss.AddVisualShape(truss_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

bar_shape = chrono.ChCylinderShape()
bar_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.2)
bar_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0.5, 0, -0.2)
bar_shape.GetCylinderGeometry().rad = 0.02
bar.AddVisualShape(bar_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))

gear_shape = chrono.ChCylinderShape()
gear_shape.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -0.05)
gear_shape.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, 0.05)
gear_shape.GetCylinderGeometry().rad = 0.1
gear1.AddVisualShape(gear_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
gear2.AddVisualShape(gear_shape, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))


application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()