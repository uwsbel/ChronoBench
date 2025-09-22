import pychrono.core as chrono
import pychrono.irrlicht as irr


system = chrono.ChSystemNSC()


truss = chrono.ChBodyEasyBox(0.2, 0.02, 0.2, 1000, True, True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetBodyFixed(True)
system.Add(truss)


arm = chrono.ChBodyEasyBox(0.5, 0.02, 0.05, 1000, True, True)
arm.SetPos(chrono.ChVectorD(0, 0.1, 0))
system.Add(arm)


gear1_radius = 0.1
gear2_radius = 0.15


gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.05, 1000, True, True)
gear1.SetPos(chrono.ChVectorD(0.2, gear1_radius, 0))
system.Add(gear1)


gear2 = chrono.ChBodyEasyCylinder(gear2_radius, 0.05, 1000, True, True)
gear2.SetPos(chrono.ChVectorD(0.4, gear2_radius, 0))
system.Add(gear2)



joint_gear1 = chrono.ChLinkRevolute()
joint_gear1.Initialize(gear1, truss, chrono.ChCoordsysD(gear1.GetPos(), chrono.Q_from_AngZ(0)))
system.Add(joint_gear1)


joint_gear2 = chrono.ChLinkRevolute()
joint_gear2.Initialize(gear2, truss, chrono.ChCoordsysD(gear2.GetPos(), chrono.Q_from_AngZ(0)))
system.Add(joint_gear2)


motor_gear1 = chrono.ChLinkMotorRotationSpeed()
motor_gear1.Initialize(gear1, truss, chrono.ChCoordsysD(gear1.GetPos(), chrono.Q_from_AngZ(0)))
motor_gear1.SetSpeed(2.0)  
system.Add(motor_gear1)


gear_constraint = chrono.ChLinkGear()
gear_constraint.Initialize(gear1, gear2, chrono.ChFrameD(gear1.GetPos()), chrono.ChFrameD(gear2.GetPos()))
system.Add(gear_constraint)


application = irr.ChIrrApp(system, "Epicyclic Gear System", irr.dimension2du(800, 600))
application.AddTypicalCamera(irr.vector3df(0, 0.2, 0.5))
application.SetContactMethod(irr.ChContactMethod_NSC)
application.SetChaseCamera(gear2.GetPos(), 0.5, 0.3)


material_gear1 = irr.ChVisualMaterial()
material_gear1.SetDiffuseColor(irr.SColor(255, 200, 0, 0))
gear1.GetVisualShape(0).SetMaterial(material_gear1)

material_gear2 = irr.ChVisualMaterial()
material_gear2.SetDiffuseColor(irr.SColor(255, 0, 200, 0))
gear2.GetVisualShape(0).SetMaterial(material_gear2)

material_arm = irr.ChVisualMaterial()
material_arm.SetDiffuseColor(irr.SColor(255, 0, 0, 200))
arm.GetVisualShape(0).SetMaterial(material_arm)


application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(0.01)