import chrono as ch
import chrono.irrlicht as irr

# Initialize PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = irr.ChIrrApp()

# Set the Irrlicht application window title
app.SetWindowTitle('Crank-Slider Mechanism Simulation')

# Enable default Irrlicht camera control
app.SetCameraController()

# Create the simulation system
system = ch.ChSystem()

# Create the floor (truss)
floor_body = ch.ChBodyEasyBox(100, 0.1, 100, 1000, False, False, False)
floor_body.SetPos(ch.ChVector3d(0, -5, 0))
floor_body.SetBodyFixed(True)
system.Add(floor_body)

# Create the crankshaft
crankshaft_body = ch.ChBodyEasyCylinder(0.1, 1, 32, False, False, False)
crankshaft_body.SetPos(ch.ChVector3d(0, 0, 0))
crankshaft_body.SetRot(ch.ChQuaternion(0, 0, 0, 1))
system.Add(crankshaft_body)

# Create the connecting rod
connecting_rod_body = ch.ChBodyEasyCylinder(0.05, 2, 32, False, False, False)
connecting_rod_body.SetPos(ch.ChVector3d(1, 0, 0))
system.Add(connecting_rod_body)

# Create the piston
piston_body = ch.ChBodyEasyBox(0.2, 0.2, 0.2, 1000, False, False, False)
piston_body.SetPos(ch.ChVector3d(2, 0, 0))
system.Add(piston_body)

# Create joints
crankshaft_joint = ch.ChLinkLockRevolute()
crankshaft_joint.Initialize(crankshaft_body, floor_body, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaternion(0, 0, 0, 1)))
system.Add(crankshaft_joint)

connecting_rod_joint1 = ch.ChLinkLockSpherical()
connecting_rod_joint1.Initialize(connecting_rod_body, crankshaft_body, ch.ChFrame3d(ch.ChVector3d(0.5, 0, 0), ch.ChQuaternion(0, 0, 0, 1)))
system.Add(connecting_rod_joint1)

connecting_rod_joint2 = ch.ChLinkLockUniversal()
connecting_rod_joint2.Initialize(connecting_rod_body, piston_body, ch.ChFrame3d(ch.ChVector3d(1, 0, 0), ch.ChQuaternion(0, 0, 0, 1)), ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaternion(0, 0, 0, 1)))
system.Add(connecting_rod_joint2)

# Create a motor to drive the crankshaft
motor = ch.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft_body, floor_body, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaternion(0, 0, 0, 1)))
motor.SetSpeedFunction(ch.ChFunction_Const(1))  # Constant angular speed
system.Add(motor)

# Visualization settings
irr.AddCameraSettings(app, ch.ChVector3d(0, -10, 5), ch.ChVector3d(0, 0, 0))

# Add Irrlicht visual shapes
ch.ChVisualShapeBox(floor_body, ch.ChFrame3d(ch.ChVector3d(0, -0.05, 0)), ch.ChVector3d(50, 0.1, 50), ch.ChColor(0.2, 0.4, 0.2)).SetTexture(ch.GetChronoDataFile('textures/brick1.jpg'))

ch.ChVisualShapeCylinder(crankshaft_body, ch.ChFrame3d(ch.ChVector3d(0, 0, 0)), 0.1, 1, 32).SetColor(ch.ChColor(0.8, 0.2, 0.2))

ch.ChVisualShapeCylinder(connecting_rod_body, ch.ChFrame3d(ch.ChVector3d(0, 0, 0)), 0.05, 2, 32).SetColor(ch.ChColor(0.2, 0.8, 0.2))

ch.ChVisualShapeBox(piston_body, ch.ChFrame3d(ch.ChVector3d(0, 0, 0)), ch.ChVector3d(0.2, 0.2, 0.2), ch.ChColor(0.2, 0.2, 0.8)).SetTexture(ch.GetChronoDataFile('textures/white.jpg'))

# Run the simulation
app.SetSimulation(system)
app.SetTimestep(0.01)
app.DoRun()

# Exit
ch.ChEngine.Exit()