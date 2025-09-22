import chrono as ch
import chrono.irrlicht as irr


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


app.SetWindowTitle("Crank-Slider Mechanism Simulation")


system = ch.ChSystem()


floor_body = ch.ChBodyEasyBox(10, 0.1, 10, 1000, False, False, False)
floor_body.SetPos(ch.ChVector3d(0, -5, 0))
floor_body.SetBodyFixed(True)
system.AddBody(floor_body)


crankshaft_body = ch.ChBodyEasyCylinder(0.1, 1.0, 1000, 20, False, False)
crankshaft_body.SetPos(ch.ChVector3d(0, 0, 0))
crankshaft_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
system.AddBody(crankshaft_body)


connecting_rod_body = ch.ChBodyEasyCylinder(0.05, 2.0, 1000, 20, False, False)
connecting_rod_body.SetPos(ch.ChVector3d(1, 0, 0))
connecting_rod_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
system.AddBody(connecting_rod_body)


piston_body = ch.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, False, False, False)
piston_body.SetPos(ch.ChVector3d(2, 0, 0))
piston_body.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
system.AddBody(piston_body)


crankshaft_revolute_joint = ch.ChRevoluteJoint(floor_body, crankshaft_body, ch.ChFrame3d(0, 0, 0, ch.ChQuaterniond(0, 0, 0, 1)))
system.AddJoint(crankshaft_revolute_joint)

connecting_rod_spherical_joint1 = ch.ChSphericalJoint(crankshaft_body, connecting_rod_body, ch.ChFrame3d(0.5, 0, 0, ch.ChQuaterniond(0, 0, 0, 1)))
system.AddJoint(connecting_rod_spherical_joint1)

connecting_rod_spherical_joint2 = ch.ChSphericalJoint(connecting_rod_body, piston_body, ch.ChFrame3d(0, 0, 0, ch.ChQuaterniond(0, 0, 0, 1)))
system.AddJoint(connecting_rod_spherical_joint2)

piston_prismatic_joint = ch.ChPrismaticJoint(floor_body, piston_body, ch.ChFrame3d(2, 0, 0, ch.ChQuaterniond(0, 0, 0, 1)), ch.ChVector3d(1, 0, 0))
system.AddJoint(piston_prismatic_joint)


crankshaft_motor = ch.ChFunction_Const(1.0)  
crankshaft_revolute_joint.SetMotorFunction(crankshaft_motor)


irr.SetEnvTexture("textures/skybox.jpg")
irr.AddLogo("chrono_logo.png")


irr.GetCamera().SetPosition(ch.ChVector3d(0, 5, -10))
irr.GetCamera().LookAt(ch.ChVector3d(0, 0, 0))


app.AddChSystem(system)
app.SetStepDuration(0.02)
app.SetRenderMode(irr.RM_SOLID)
app.Run()