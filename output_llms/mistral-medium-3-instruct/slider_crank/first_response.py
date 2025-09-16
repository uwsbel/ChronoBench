import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Set the path to the Chrono data directory
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Update this path

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# Create the floor (truss)
floor_body = chrono.ChBody()
floor_body.SetBodyFixed(True)
floor_body.SetCollide(True)
floor_body.GetCollisionModel().AddBox(2, 0.1, 2)  # Dimensions of the floor
floor_body.GetVisualModel().AddBox(2, 0.1, 2)
system.Add(floor_body)

# Create the crankshaft
crank = chrono.ChBody()
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVectorD(0.1, 0.1, 0.1))
crank.SetPos(chrono.ChVectorD(0, 0.5, 0))
crank.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
crank.GetCollisionModel().AddCylinder(0.1, 0.2)  # Radius, height
crank.GetVisualModel().AddCylinder(0.1, 0.2)
system.Add(crank)

# Create the connecting rod
rod = chrono.ChBody()
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVectorD(0.05, 0.01, 0.01))
rod.SetPos(chrono.ChVectorD(0.5, 0.5, 0))
rod.GetCollisionModel().AddBox(0.5, 0.05, 0.05)  # Dimensions
rod.GetVisualModel().AddBox(0.5, 0.05, 0.05)
system.Add(rod)

# Create the piston
piston = chrono.ChBody()
piston.SetMass(0.3)
piston.SetInertiaXX(chrono.ChVectorD(0.01, 0.01, 0.01))
piston.SetPos(chrono.ChVectorD(1.0, 0.5, 0))
piston.GetCollisionModel().AddBox(0.1, 0.1, 0.1)  # Dimensions
piston.GetVisualModel().AddBox(0.1, 0.1, 0.1)
system.Add(piston)

# Create joints
# Revolute joint between floor and crank (ground joint)
joint_crank_ground = chrono.ChLinkLockRevolute()
joint_crank_ground.Initialize(floor_body, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)))
system.Add(joint_crank_ground)

# Revolute joint between crank and rod
joint_crank_rod = chrono.ChLinkLockRevolute()
joint_crank_rod.Initialize(crank, rod, chrono.ChCoordsysD(chrono.ChVectorD(0.2, 0.5, 0)))
system.Add(joint_crank_rod)

# Revolute joint between rod and piston
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0.5, 0)))
system.Add(joint_rod_piston)

# Prismatic joint between piston and floor (constraining piston to move along x-axis)
joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(floor_body, piston, chrono.ChCoordsysD(chrono.ChVectorD(1.0, 0.5, 0), chrono.Q_from_AngX(chrono.CH_PI_2)))
system.Add(joint_piston_ground)

# Create a motor to drive the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crank, floor_body, chrono.ChFrameD(chrono.ChVectorD(0, 0.5, 0)))
motor.SetSpeedFunction(chrono.ChFunction_Const(1.0))  # 1 rad/s constant speed
system.Add(motor)

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1, 2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1.5, 5, -2.5), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# Set some visualization properties
vis.SetSymbolScale(0.02)
vis.SetShadows(True)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)