import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#
# ---------------------------------------------------------------------

mysystem = chrono.ChSystemNSC()

# Set gravitational acceleration
mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the floor
floor = chrono.ChBodyEasyBox(2, 0.1, 2, 1000, True, True)
floor.SetPos(chrono.ChVectorD(0, -1, 0))
mysystem.Add(floor)

# Create the crankshaft
crankshaft_radius = 0.2
crankshaft = chrono.ChBodyEasyCylinder(crankshaft_radius, 0.2, 1000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0, 0))
mysystem.Add(crankshaft)

# Create the connecting rod
connecting_rod_length = 1
connecting_rod = chrono.ChBodyEasyBox(0.1, connecting_rod_length / 2, 0.1, 1000, True, True)
connecting_rod.SetPos(chrono.ChVectorD(crankshaft_radius, 0, 0))
mysystem.Add(connecting_rod)

# Create the piston
piston_radius = 0.1
piston = chrono.ChBodyEasyCylinder(piston_radius, 0.1, 1000, True, True)
piston.SetPos(chrono.ChVectorD(crankshaft_radius + connecting_rod_length, 0, 0))
mysystem.Add(piston)

# ---------------------------------------------------------------------
#
#  Create joints
#
# ---------------------------------------------------------------------

# Revolute joint between crankshaft and floor
revolute_joint_crank = chrono.ChLinkRevolute()
revolute_joint_crank.Initialize(floor, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0)))
mysystem.Add(revolute_joint_crank)

# Revolute joint between connecting rod and crankshaft
revolute_joint_rod_crank = chrono.ChLinkRevolute()
revolute_joint_rod_crank.Initialize(crankshaft, connecting_rod, chrono.ChCoordsysD(chrono.ChVectorD(crankshaft_radius, 0, 0)))
mysystem.Add(revolute_joint_rod_crank)

# Revolute joint between connecting rod and piston
revolute_joint_rod_piston = chrono.ChLinkRevolute()
revolute_joint_rod_piston.Initialize(connecting_rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(connecting_rod_length, 0, 0)))
mysystem.Add(revolute_joint_rod_piston)

# ---------------------------------------------------------------------
#
#  Add motor to the crankshaft
#
# ---------------------------------------------------------------------

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, floor, chrono.ChVectorD(0, 0, 0))
motor.SetSpeedFunction(chrono.ChFunction_Const(2 * math.pi))  # Set angular speed to 2*pi rad/s
mysystem.Add(motor)

# ---------------------------------------------------------------------
#
#  Create Irrlicht visualization
#
# ---------------------------------------------------------------------

vis = chronoirr.ChIrrApp(mysystem, "Crank-Slider Mechanism", chronoirr.dimension2du(800, 600))
vis.AddTypicalSky()
vis.AddTypicalCamera(chrono.ChVectorD(2, 2, 2))
vis.AddLightWithShadow(chrono.ChVectorD(2, 5, 2), chrono.ChVectorD(0.8, 0.8, 0.8), 10)

vis.SetCameraPosition(chrono.ChVectorD(2, 2, 2))
vis.SetCameraDirection(chrono.ChVectorD(-2, -2, -2))

# Customize visualization (add textures, logos, etc.)

# ---------------------------------------------------------------------
#
#  Run the simulation
#
# ---------------------------------------------------------------------

while vis.GetDevice().run():
    vis.BeginScene()
    vis.Render()
    vis.DoStep()
    vis.EndScene()