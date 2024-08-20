import pychrono as chrono
import pychrono.irrlicht as irr
import pychrono.postprocess as postprocess

# Initialize the ChSystem
system = chrono.ChSystemNSC()

# Set the default collision system
collision_model = chrono.ChCollisionModelParallel()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the floor (truss)
floor_body = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
floor_body.SetPos(chrono.ChVectorD(0, -1, 0))
floor_body.SetBodyFixed(True)
system.Add(floor_body)

# Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(0.1, 0.5, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
crankshaft.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(1, 0, 0)))
system.Add(crankshaft)

# Create the connecting rod
rod = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000, True, True)
rod.SetPos(chrono.ChVectorD(0.5, 0, 0))
system.Add(rod)

# Create the piston
piston = chrono.ChBodyEasyCylinder(0.2, 0.5, 1000, True, True)
piston.SetPos(chrono.ChVectorD(1, 0, 0))
system.Add(piston)

# Define revolute joint for the crankshaft
crank_joint = chrono.ChLinkRevolute()
crank_joint.Initialize(crankshaft, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
system.Add(crank_joint)

# Add a motor to the crankshaft
crank_motor = chrono.ChLinkMotorRotationSpeed()
crank_motor.Initialize(crankshaft, floor_body, chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
crank_motor.SetMotorFunction(chrono.ChFunction_Const(1))  # 1 rad/s
system.Add(crank_motor)

# Define the revolute joint between crankshaft and connecting rod
crank_rod_joint = chrono.ChLinkRevolute()
crank_rod_joint.Initialize(crankshaft, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)))
system.Add(crank_rod_joint)

# Define the revolute joint between connecting rod and piston
rod_piston_joint = chrono.ChLinkRevolute()
rod_piston_joint.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1, 0, 0)))
system.Add(rod_piston_joint)

# Define the prismatic joint for the piston
piston_joint = chrono.ChLinkLockPrismatic()
piston_joint.Initialize(piston, floor_body, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.ChVectorD(0, 1, 0))))
system.Add(piston_joint)

# Create the Irrlicht visualization system
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.EnableShadows()
vis.AddCamera(chrono.ChVectorD(2.5, 2.5, 2.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()

# Initialize the visualization assets
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(0, -1, 0), 5, 0.1, 10)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    system.DoStepDynamics(0.01)
    vis.EndScene()

# Finalize the simulation
vis.GetDevice().closeDevice()