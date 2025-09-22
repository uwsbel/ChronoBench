importchrono
importchrono.core as chrono
importchrono irrlicht as chronoirr
importchrono postprocess as chronop
importchrono visualization as chronovis


chrono.ChSystem.SetChronoVerbose(False)
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


vis = chronovis.ChVisualSystemIrrlicht()
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.SetWindowSize(800, 600)
vis.SetNumberOfFrames(60)
vis.SetCameraPosition(chrono.ChVectorD(0, 3, 3))
vis.SetCameraRotation(chrono.ChVectorD(0, 0, 0))
vis.AddLight(chrono.ChVectorD(0, 5, 5), chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(0, -5, 5), chrono.ChColor(1, 1, 1))
vis.AddLight(chrono.ChVectorD(0, 0, 5), chrono.ChColor(1, 1, 1))
vis.SetLightIntensity(1)
vis.SetAmbientIntensity(0.5)


floor = chrono.ChBody()
floor.SetPos(chrono.ChVectorD(0, 0, 0))
floor.SetBodyFixed(True)
floor_shape = chrono.ChTruss()
floor_shape.GetPoints().push_back(chrono.ChVectorD(-5, 0, -5))
floor_shape.GetPoints().push_back(chrono.ChVectorD(5, 0, -5))
floor_shape.GetPoints().push_back(chrono.ChVectorD(5, 0, 5))
floor_shape.GetPoints().push_back(chrono.ChVectorD(-5, 0, 5))
floor.AddVisualShape(floor_shape)
sys.AddBody(floor)


crank = chrono.ChBody()
crank.SetPos(chrono.ChVectorD(0, 0, 0))
crank.SetMass(1)
crank.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crank_shape = chrono.ChCylinder()
crank_shape.SetHeight(0.2)
crank_shape.SetRadius(0.1)
crank_shape.SetPos(chrono.ChVectorD(0, 0, 0))
crank_shape.SetRotation(chrono.ChVectorD(0, 1, 0))
crank.AddVisualShape(crank_shape)
sys.AddBody(crank)


motor = chrono.ChMotorRotation()
motor.SetName('crank_motor')
motor.SetRotDir(chrono.ChVectorD(0, 1, 0))
motor.SetSpeed(2)  
crank.AddForce(motor)


rod = chrono.ChBody()
rod.SetPos(chrono.ChVectorD(0.2, 0, 0))
rod.SetMass(1)
rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
rod_shape = chrono.ChBox()
rod_shape.SetLengths(chrono.ChVectorD(0.2, 0.1, 0.1))
rod_shape.SetPos(chrono.ChVectorD(0, 0, 0))
rod.AddVisualShape(rod_shape)
sys.AddBody(rod)


joint1 = chrono.ChLinkLockSpherical()
joint1.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(joint1)


piston = chrono.ChBody()
piston.SetPos(chrono.ChVectorD(0.4, 0, 0))
piston.SetMass(1)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
piston_shape = chrono.ChCylinder()
piston_shape.SetHeight(0.2)
piston_shape.SetRadius(0.1)
piston_shape.SetPos(chrono.ChVectorD(0, 0, 0))
piston.AddVisualShape(piston_shape)
sys.AddBody(piston)


joint2 = chrono.ChLinkLockSpherical()
joint2.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(joint2)


slider = chrono.ChLinkLockRevolute()
slider.Initialize(piston, floor, chrono.ChCoordsysD(chrono.ChVectorD(0.4, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
slider.SetRotationAxis(chrono.ChVectorD(1, 0, 0))
sys.AddLink(slider)


logo = chrono.ChBody()
logo.SetPos(chrono.ChVectorD(0, 0.1, 0))
logo.SetBodyFixed(True)
logo_shape = chrono.ChTexture()
logo_shape.SetTextureFilename("chrono_data/chrono.png")
logo_shape.SetPos(chrono.ChVectorD(0, 0, 0))
logo_shape.SetRotation(chrono.ChVectorD(0, 0, 90))
logo.AddVisualShape(logo_shape)
sys.AddBody(logo)


vis.AttachSystem(sys)
vis.SetRenderMode(chronovis.RM_OPENGL)
vis.BeginAnimation()
for i in range(1000):
    sys.DoStepDynamics(0.001)
    vis.Render()
    vis.Wait()
vis.EndAnimation()