import chrono as ch
import chrono.irrlicht as chirr

# Initialize PyChrono environment
ch.CH_Environment.SetSimulationProfilerEnabled(ch.C profiler_mode)

# Create the Chrono system
system = ch.ChSystemNSC()

# Create the Irrlicht application
app = chirr.ChIrrApp(system, 'Crank-Slider Mechanism')

# Create the floor (truss)
floor = ch.ChBodyEasyFloor('floor')
floor.SetPos(ch.ChVector3d(0, -5, 0))
system.AddBody(floor)

# Create the crankshaft
crankshaft = ch.ChBody()
crankshaft.SetPos(ch.ChVector3d(0, 0, 0))
crankshaft.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
crankshaft.EnableCollisionShape(True)
crankshaft_shape = ch.ChBoxShape(crankshaft, ch.ChVector3d(0.5, 0.1, 0.1), ch.ChVector3d(0, 0, 0))
crankshaft_shape.SetColor(ch.ChColor(1, 0, 0))
crankshaft.AddVisualShape(crankshaft_shape)
system.AddBody(crankshaft)

# Create the connecting rod
connecting_rod = ch.ChBody()
connecting_rod.SetPos(ch.ChVector3d(2, 0, 0))
connecting_rod.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
connecting_rod.EnableCollisionShape(True)
connecting_rod_shape = ch.ChBoxShape(connecting_rod, ch.ChVector3d(1, 0.1, 0.1), ch.ChVector3d(0, 0, 0))
connecting_rod_shape.SetColor(ch.ChColor(0, 1, 0))
connecting_rod.AddVisualShape(connecting_rod_shape)
system.AddBody(connecting_rod)

# Create the piston
piston = ch.ChBody()
piston.SetPos(ch.ChVector3d(4, 0, 0))
piston.SetRot(ch.ChQuaterniond(0, 0, 0, 1))
piston.EnableCollisionShape(True)
piston_shape = ch.ChBoxShape(piston, ch.ChVector3d(0.5, 0.1, 0.1), ch.ChVector3d(0, 0, 0))
piston_shape.SetColor(ch.ChColor(0, 0, 1))
piston.AddVisualShape(piston_shape)
system.AddBody(piston)

# Create joints
crankshaft_joint = ch.ChLinkLockPrizmaticLock(crankshaft, connecting_rod, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(0, 0, 0, 1)))
connecting_rod_joint = ch.ChLinkLockPrizmaticLock(connecting_rod, piston, ch.ChFrame3d(ch.ChVector3d(2, 0, 0), ch.ChQuaterniond(0, 0, 0, 1)))
system.AddLink(crankshaft_joint)
system.AddLink(connecting_rod_joint)

# Create motor to spin crankshaft
crankshaft_motor = ch.ChLinkMotorRotationSpeed()
crankshaft_motor.Initialize(crankshaft, floor, ch.ChFrame3d(ch.ChVector3d(0, 0, 0), ch.ChQuaterniond(0, 0, 0, 1)))
crankshaft_motor.SetSpeedFunction(ch.ChFunction_Const(1))
system.AddLink(crankshaft_motor)

# Set default parameters
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))

# Visualization settings
app.SetCameraVertical(ch.ChVector3d(0, 0, 1))
app.SetCameraTarget(ch.ChVector3d(0, 0, 0))
app.SetCameraDistance(10)

# Run simulation
app.SetMaxSimFPS(1000)
app.Run()