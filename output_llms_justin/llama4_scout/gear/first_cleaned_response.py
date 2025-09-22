import chrono as ch
import math


ch.ChEngine.Initialize()


app = ch.ChIrrApp(ch.ChSystem(), "Epicyclic Gear Simulation")
app.SetTimestep(0.01)
app.SetVerbose(True)


truss_body = ch.ChBody()
truss_body.SetBodyFixed(True)
truss_shape = ch.ChBox(2, 0.1, 0.1)
truss_shape.SetColor(ch.ChColor(0.7, 0.7, 0.7))
truss_body.AddVisualShape(truss_shape, ch.ChFrame(truss_body.GetPos(), ch.ChQuaternion(0, 0, 0, 1)))
app.GetSystem().Add(truss_body)


truss_body.SetPos(ch.ChVector(0, 0, 0))


bar_body = ch.ChBody()
bar_shape = ch.ChBox(1, 0.05, 0.05)
bar_shape.SetColor(ch.ChColor(0.5, 0.5, 0.5))
bar_body.AddVisualShape(bar_shape, ch.ChFrame(bar_body.GetPos(), ch.ChQuaternion(0, 0, 0, 1)))
app.GetSystem().Add(bar_body)


bar_body.SetPos(ch.ChVector(0, 1, 0))


gear_motor_body = ch.ChBody()
gear_motor_shape = ch.ChCylinder(0.2, 0.05)
gear_motor_shape.SetColor(ch.ChColor(0.2, 0.8, 0.2))
gear_motor_body.AddVisualShape(gear_motor_shape, ch.ChFrame(gear_motor_body.GetPos(), ch.ChQuaternion(0, 0, 0, 1)))
app.GetSystem().Add(gear_motor_body)


gear_motor_body.SetPos(ch.ChVector(-1, 0, 0))


gear_motor = ch.ChRotationalMotor()
gear_motor.SetMechLink(gear_motor_body, bar_body)
gear_motor.SetFunction(ch.ChFunction_Const(1))  
app.GetSystem().Add(gear_motor)


gear1_body = ch.ChBody()
gear1_shape = ch.ChCylinder(0.3, 0.05)
gear1_shape.SetColor(ch.ChColor(0.8, 0.2, 0.2))
gear1_body.AddVisualShape(gear1_shape, ch.ChFrame(gear1_body.GetPos(), ch.ChQuaternion(0, 0, 0, 1)))
app.GetSystem().Add(gear1_body)

gear2_body = ch.ChBody()
gear2_shape = ch.ChCylinder(0.4, 0.05)
gear2_shape.SetColor(ch.ChColor(0.2, 0.2, 0.8))
gear2_body.AddVisualShape(gear2_shape, ch.ChFrame(gear2_body.GetPos(), ch.ChQuaternion(0, 0, 0, 1)))
app.GetSystem().Add(gear2_body)


gear1_body.SetPos(ch.ChVector(0.5, 0.5, 0))
gear2_body.SetPos(ch.ChVector(0.5, -0.5, 0))


gear1_joint = ch.ChRevoluteJoint()
gear1_joint.SetMechLink(gear1_body, truss_body)
gear1_joint.SetPos(ch.ChVector(0.5, 0.5, 0))
app.GetSystem().Add(gear1_joint)

gear2_joint = ch.ChRevoluteJoint()
gear2_joint.SetMechLink(gear2_body, truss_body)
gear2_joint.SetPos(ch.ChVector(0.5, -0.5, 0))
app.GetSystem().Add(gear2_joint)


contact = ch.ChContactContainer()
app.GetSystem().Add(contact)


app.AssetBindAll()
app.AssetUpdateAll()


while app.Run():
    app.GetSystem().DoStepDynamics(0.01)