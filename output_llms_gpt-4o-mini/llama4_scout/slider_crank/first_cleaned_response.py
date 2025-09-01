import pychrono as chrono
import pychrono.irrlicht as irr


chrono.ChEngine.Initialize()


app = irr.IrrlichtApplication()


step_size = 0.01
fps = 60


floor_body = chrono.ChBodyStatic()
floor_shape = chrono.ChBox(10, 0.1, 10)
floor_body.AddShape(floor_shape)
floor_body.SetPos(chrono.ChVector3d(0, -5, 0))
floor_body.SetFixed(True)


app.GetSystem().Add(floor_body)


crankshaft_body = chrono.ChBody()
crankshaft_shape = chrono.ChBox(0.5, 0.1, 0.1)
crankshaft_body.AddShape(crankshaft_shape)
crankshaft_body.SetPos(chrono.ChVector3d(0, 0, 0))
crankshaft_body.SetMass(1.0)


app.GetSystem().Add(crankshaft_body)


connecting_rod_body = chrono.ChBody()
connecting_rod_shape = chrono.ChBox(1.0, 0.1, 0.1)
connecting_rod_body.AddShape(connecting_rod_shape)
connecting_rod_body.SetPos(chrono.ChVector3d(2, 0, 0))
connecting_rod_body.SetMass(0.5)


app.GetSystem().Add(connecting_rod_body)


piston_body = chrono.ChBody()
piston_shape = chrono.ChBox(0.5, 0.1, 0.1)
piston_body.AddShape(piston_shape)
piston_body.SetPos(chrono.ChVector3d(4, 0, 0))
piston_body.SetMass(0.2)


app.GetSystem().Add(piston_body)


crankshaft_joint = chrono.ChLinkLockRevolute()
crankshaft_joint.Init(floor_body, crankshaft_body, chrono.ChVector3d(0, 0, 0))
app.GetSystem().Add(crankshaft_joint)


connecting_rod_joint = chrono.ChLinkLockSpherical()
connecting_rod_joint.Init(crankshaft_body, connecting_rod_body, chrono.ChVector3d(0.5, 0, 0))
app.GetSystem().Add(connecting_rod_joint)


piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Init(connecting_rod_body, piston_body, chrono.ChVector3d(1, 0, 0), chrono.ChVector3d(0, 1, 0))
app.GetSystem().Add(piston_joint)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Init(floor_body, crankshaft_body, chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 1), 10)
app.GetSystem().Add(motor)


app.SetCamera(chrono.ChVector3d(10, 10, 10), chrono.ChVector3d(0, 0, 0))
app.SetLight(chrono.ChVector3d(10, 10, 10))


app.AddLogo(chrono.ChLogo('chrono_logo.png'))
app.AddTexture('floor', chrono.ChTexture('floor.jpg'))

while app.Run() and not app.WasKeyPressed(irr.KEY_ESCAPE):
    app.GetSystem().DoStepDynamics(step_size)

app.End()