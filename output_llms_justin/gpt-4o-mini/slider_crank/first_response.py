import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

# Initialize the PyChrono system
chrono.ChSystemNSC.SetNumThreads(1)
system = chrono.ChSystemNSC()

# Create the Irrlicht visualization system
application = chronoirr.ChIrrApp(system, "Crank-Slider Mechanism", chronoirr.dimension2du(800, 600))
application.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 1, -5), chrono.ChVectorD(0, 0, 0))
application.AddLight(chrono.ChVectorD(10, 10, 10), chrono.ChVectorD(1, 1, 1))

# Create the ground (truss)
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
ground.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(ground)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(0.05, 1.0, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(crankshaft)

# Create the connecting rod
connecting_rod = chrono.ChBodyEasyBox(0.02, 0.5, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
connecting_rod.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(connecting_rod)

# Create the piston
piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1.0, 0.1, 0))
piston.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
system.Add(piston)

# Create joints
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crankshaft, ground, chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))
system.Add(crank_joint)

rod_joint = chrono.ChLinkLockSpherical()
rod_joint.Initialize(connecting_rod, crankshaft, chrono.ChVectorD(0.5, 0.25, 0))
system.Add(rod_joint)

piston_joint = chrono.ChLinkLockSpherical()
piston_joint.Initialize(piston, connecting_rod, chrono.ChVectorD(1.0, 0.1, 0))
system.Add(piston_joint)

# Set motor to crankshaft
motor = chrono.ChLinkMotorRotationTorque()
motor.Initialize(crankshaft, ground, chrono.ChVectorD(0, 0, 0))
motor.SetMotorFunction(chrono.ChFunction_Const(10))  # Constant torque
system.Add(motor)

# Set initial conditions
crankshaft.SetRot(chrono.Q_from_AngZ(0))  # Start at angle 0
connecting_rod.SetPos(chrono.ChVectorD(0.5, 0.25, 0))
piston.SetPos(chrono.ChVectorD(1.0, 0.1, 0))

# Simulation loop
application.SetTimestep(0.01)
while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    system.DoStepDynamics(application.GetTimestep())

# Clean up
application.GetDevice().drop()