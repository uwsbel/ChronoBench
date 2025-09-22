import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add the floor
#
# ---------------------------------------------------------------------

mysystem = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the floor as a truss element
floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
floor.SetBodyFixed(True)
mysystem.Add(floor)

# ---------------------------------------------------------------------
#
#  Create the crank-slider mechanism
#
# ---------------------------------------------------------------------

# Crankshaft
crank_radius = 0.5
crank = chrono.ChBodyEasyCylinder(crank_radius, 0.1, 1000, True, True)
crank.SetPos(chrono.ChVectorD(0, 0, 0))
mysystem.Add(crank)

# Motor to drive the crankshaft
motor = chrono.ChLinkMotorRotationSpeed()
motor.SetBody1(crank)
motor.SetAngleFunction(chrono.ChFunction_Const(chrono.CH_C_PI))
motor.SetSpeed(10)  # Angular speed in rad/s
mysystem.Add(motor)

# Connecting rod
rod_length = 1.0
rod = chrono.ChBodyEasyBox(rod_length, 0.05, 0.05, 1000, True, True)
rod.SetPos(chrono.ChVectorD(crank_radius, 0, 0))
mysystem.Add(rod)

# Piston
piston = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(crank_radius + rod_length, 0, 0))
mysystem.Add(piston)

# Revolute joints to connect the components
crank_rod_joint = chrono.ChLinkRevolute()
crank_rod_joint.Initialize(crank, rod, chrono.ChVectorD(crank_radius, 0, 0))
mysystem.Add(crank_rod_joint)

rod_piston_joint = chrono.ChLinkRevolute()
rod_piston_joint.Initialize(rod, piston, chrono.ChVectorD(rod_length, 0, 0))
mysystem.Add(rod_piston_joint)

# ---------------------------------------------------------------------
#
#  Set up Irrlicht visualization
#
# ---------------------------------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(mysystem)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Crank-Slider Mechanism")

# Customize camera settings
camera = vis.GetCamera()
camera.SetPos(chrono.ChVectorD(2, 2, 2))
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))

# Add lighting and other visual elements
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(1, 1, 1), 50)

# Start the simulation loop
while vis.Run():
    mysystem.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()