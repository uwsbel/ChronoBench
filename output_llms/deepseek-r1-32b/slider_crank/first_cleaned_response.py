importchrono
importchrono.core as core
importchrono.irrlicht as irr
importchrono.postprocess as post


system = core.ChSystemNSC()
system.Set_G_acc(core.ChVectorD(0, -9.81, 0))


application = irr.ChIrrApp(system, "Crank-Slider Mechanism", irr.CHIRR_APP_OPTIONS_DEFAULT)
application.SetWindowSize(core.ChVector2i(800, 600))
application.SetCameraPosition(core.ChVectorD(0, 3, 3))
application.SetCameraLookAt(core.ChVectorD(0, 0, 0))
application.AddTypicalLights()


floor = core.ChBody()
floor.SetBodyFixed(True)
floor.SetPos(core.ChVectorD(0, 0, 0))
floor_shape = core.ChTruss()
floor_shape.SetPoints(core.ChVectorD(0, 0, 0), core.ChVectorD(2, 0, 0))
floor_shape.SetPoints(core.ChVectorD(0, 0, 0), core.ChVectorD(0, 0, 2))
floor.AddVisualShape(floor_shape)
system.AddBody(floor)


crank = core.ChBody()
crank.SetPos(core.ChVectorD(0, 0, 0))
crank_shape = core.ChCylinder()
crank_shape.SetRadius(0.1)
crank_shape.SetHeight(0.2)
crank.AddVisualShape(crank_shape)
system.AddBody(crank)


motor = core.ChMotorRotationAngular()
motor.SetRotDir(core.ChVectorD(0, 1, 0))
motor.SetVelocity(2)  
crank.AddForce(motor)


rod = core.ChBody()
rod.SetPos(core.ChVectorD(0.2, 0, 0))
rod_shape = core.ChBox()
rod_shape.SetDimensions(core.ChVectorD(0.1, 0.05, 0.05))
rod.AddVisualShape(rod_shape)
system.AddBody(rod)


piston = core.ChBody()
piston.SetPos(core.ChVectorD(0.4, 0, 0))
piston_shape = core.ChCylinder()
piston_shape.SetRadius(0.05)
piston_shape.SetHeight(0.2)
piston.AddVisualShape(piston_shape)
system.AddBody(piston)


crank_joint = core.ChLinkRev()
crank_joint.Initialize(crank, floor, core.ChCoordsysD(core.ChVectorD(0, 0, 0), core.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(crank_joint)

rod_joint1 = core.ChLinkRev()
rod_joint1.Initialize(rod, crank, core.ChCoordsysD(core.ChVectorD(0.1, 0, 0), core.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(rod_joint1)

rod_joint2 = core.ChLinkRev()
rod_joint2.Initialize(rod, piston, core.ChCoordsysD(core.ChVectorD(0.1, 0, 0), core.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(rod_joint2)

piston_joint = core.ChLinkSlider()
piston_joint.Initialize(piston, floor, core.ChCoordsysD(core.ChVectorD(0.4, 0, 0), core.ChQuaternionD(1, 0, 0, 0)))
system.AddLink(piston_joint)


application.Compile()

while application.GetDevice().Run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()
    core.ChSleep(0.001)