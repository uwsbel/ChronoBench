import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.IrrlichtApplication()


system = ch.ChSystem()


truss_body = ch.ChBody()
truss_body.SetBodyFixed(True)
truss_shape = ch.ChBox(0.1, 0.1, 0.5)
truss_body.AddCollisionShape(truss_shape)
truss_body.SetPos(ch.ChVector3d(0, 0, 0))
system.AddBody(truss_body)


bar_body = ch.ChBody()
bar_shape = ch.ChBox(0.05, 0.05, 1.0)
bar_body.AddCollisionShape(bar_shape)
bar_body.SetPos(ch.ChVector3d(0, 0, 1.5))
bar_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
bar_motor = ch.ChLinkMotorRotationSpeed()
bar_motor.Initialize(truss_body, bar_body, ch.ChFrame3d(0, 0, 0), ch.ChFrame3d(0, 0, 1.5))
bar_motor.SetSpeed(1)
system.AddBody(bar_body)
system.AddLink(bar_motor)


gear1_body = ch.ChBody()
gear1_shape = ch.ChCylinder(0.2, 0.05)
gear1_body.AddCollisionShape(gear1_shape)
gear1_body.SetPos(ch.ChVector3d(0.5, 0, 1.0))
gear1_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
system.AddBody(gear1_body)

gear2_body = ch.ChBody()
gear2_shape = ch.ChCylinder(0.1, 0.05)
gear2_body.AddCollisionShape(gear2_shape)
gear2_body.SetPos(ch.ChVector3d(-0.5, 0, 1.0))
gear2_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
system.AddBody(gear2_body)


gear1_motor = ch.ChLinkMotorRotationSpeed()
gear1_motor.Initialize(truss_body, gear1_body, ch.ChFrame3d(0, 0, 0), ch.ChFrame3d(0.5, 0, 1.0))
gear1_motor.SetSpeed(2)
system.AddLink(gear1_motor)


gear_contact = ch.ChLinkContact()
gear_contact.Initialize(gear1_body, gear2_body, ch.ChFrame3d(0, 0, 0), ch.ChFrame3d(0, 0, 0))
gear_contact.SetRestitutionCoeff(0.9)
gear_contact.SetFrictionCoeff(0.5)
system.AddLink(gear_contact)


vis = irr.IrrlichtVisualSystem()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Epicyclic Gear Simulation')
vis.EnableShadows()
vis.EnableContactDrawing(True)


while vis.Run() and not vis.WasUserQuit():
    system.DoStepDynamics(0.02)

vis.Exit()