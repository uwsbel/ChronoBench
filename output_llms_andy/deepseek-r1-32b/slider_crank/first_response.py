importchrono
importchrono.core as chrono
importchrono irrlicht as chronoirr
importchrono postprocess as chronop
importchrono visualization as chronovis

# Initialize Chrono system
chrono.ChSystem.SetChronoVerbose(False)
sys = chrono.ChSystemNSC()
sys.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create Irrlicht application
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

# Add floor
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

# Add crankshaft
crank = chrono.ChBody()
crank.SetPos(chrono.ChVectorD(0, 0, 0))
crank.SetMass(1)
crank.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crank_shape = chrono.ChCylinder()
crank_shape.SetHeight(0.1)
crank_shape.SetRadius(0.2)
crank.AddVisualShape(crank_shape)
sys.AddBody(crank)

# Add motor
motor = chrono.ChMotorRotation()
motor.SetName('crank_motor')
motor.SetAngle(chrono.ChFunction_Const(0))
motor.SetSpeed(chrono.ChFunction_Const(2))  # 2 rad/s
motor.SetTorque(chrono.ChFunction_Const(0))
crank.AddForce(motor)

# Add connecting rod
rod = chrono.ChBody()
rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
rod.SetMass(1)
rod.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
rod_shape = chrono.ChBox()
rod_shape.SetLengths(chrono.ChVectorD(0.25, 0.1, 0.1))
rod.AddVisualShape(rod_shape)
sys.AddBody(rod)

# Add revolute joint between crank and rod
joint1 = chrono.ChLinkLockRevolute()
joint1.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(joint1)

# Add piston
piston = chrono.ChBody()
piston.SetPos(chrono.ChVectorD(1, 0, 0))
piston.SetMass(1)
piston.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
piston_shape = chrono.ChCylinder()
piston_shape.SetHeight(0.5)
piston_shape.SetRadius(0.1)
piston.AddVisualShape(piston_shape)
sys.AddBody(piston)

# Add revolute joint between rod and piston
joint2 = chrono.ChLinkLockRevolute()
joint2.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
sys.AddLink(joint2)

# Add slider joint for piston
slider = chrono.ChLinkLockPrismatic()
slider.Initialize(piston, floor, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)), chrono.ChVectorD(1, 0, 0))
sys.AddLink(slider)

# Add visual elements
logo = chrono.ChVisualShape()
logo.SetTexture(chrono.GetChronoDataFile('textures/chrono.png'))
logo.SetScale(chrono.ChVectorD(0.2, 0.2, 0.2))
logo.SetPos(chrono.ChVectorD(0, 0.1, 0))
crank.AddVisualShape(logo)

# Run simulation
vis.AttachSystem(sys)
vis.SetRenderMode(chronovis.RENDER_MODE_OPENGL)
vis.Render()

for _ in range(1000):
    sys.DoStepDynamics(0.01)
    vis.Render()
    if vis.GetDevice().IsKeyDown(chronoirr.KEY_ESCAPE):
        break

vis.RemoveAll()