import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set gravity
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000, True)
ground.SetBodyFixed(True)
system.Add(ground)

# Create the crankshaft
crank = chrono.ChBodyEasyBox(0.1, 0.1, 0.05, 1000, True)
crank.SetPos(chrono.ChVectorD(0, 0.1, 0))
system.Add(crank)

# Create the connecting rod
rod = chrono.ChBodyEasyBox(0.2, 0.05, 0.05, 1000, True)
rod.SetPos(chrono.ChVectorD(0.15, 0.1, 0))
system.Add(rod)

# Create the piston
piston = chrono.ChBodyEasyBox(0.05, 0.1, 0.05, 1000, True)
piston.SetPos(chrono.ChVectorD(0.3, 0.1, 0))
system.Add(piston)

# Create joints

# Revolute joint for crankshaft
crank_joint = chrono.ChLinkLockRevolute()
crank_joint.Initialize(crank, ground, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.1, 0)))
system.AddLink(crank_joint)

# Revolute joint between crank and rod
crank_rod_joint = chrono.ChLinkLockRevolute()
crank_rod_joint.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.05, 0.1, 0)))
system.AddLink(crank_rod_joint)

# Spherical joint between rod and piston
rod_piston_joint = chrono.ChLinkLockSpherical()
rod_piston_joint.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0.1, 0)))
system.AddLink(rod_piston_joint)

# Prismatic joint for piston
piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(piston, ground, chrono.ChCoordsysD(chrono.ChVectorD(0.3, 0.1, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_Z)))
system.AddLink(piston_joint)

# Add motor to drive the crank
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0.1, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(5.0))  # 5 rad/s
system.AddLink(motor)

# Create the Irrlicht visualization
application = chronoirr.ChIrrApp(system, "Crank-Slider Mechanism", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.6, 0.3, 0.8))

# Add logo (optional)
logo = chronoirr.ChIrrEffectLogo()
application.AddUserEffect(logo)

# Enable shadows
application.AddShadowAll()

# Set some visualization properties
application.AssetBindAll()
application.AssetUpdateAll()

# Simulation loop
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()