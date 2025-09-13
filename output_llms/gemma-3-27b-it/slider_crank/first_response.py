import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---
#  Create the simulation system and add items
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# ---
# Create the floor (a simple truss)
# ---
body_floor = chrono.ChBodyEasy()
body_floor.SetBodyFixed(True)
body_floor.SetPos(chrono.ChVectorD(0, -0.5, 0))
body_floor.SetShapeBoxes(0.5, 0.1, 1)
body_floor.SetCollide(True)
system.Add(body_floor)

# ---
# Create the crankshaft
# ---
crank_length = 0.3
crank_radius = 0.1
crank_mass = 1.0
crank_inertia = 0.01

body_crank = chrono.ChBodyEasy()
body_crank.SetMass(crank_mass)
body_crank.SetInertia(crank_inertia)
body_crank.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_crank.SetShapeCylinder(crank_radius, crank_length)
body_crank.SetCollide(True)
system.Add(body_crank)

# ---
# Create the connecting rod
# ---
rod_length = 0.4
rod_mass = 0.5
rod_inertia = 0.005

body_rod = chrono.ChBodyEasy()
body_rod.SetMass(rod_mass)
body_rod.SetInertia(rod_inertia)
body_rod.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_rod.SetShapeBoxes(rod_length / 2, 0.05, 0.05)
body_rod.SetCollide(True)
system.Add(body_rod)

# ---
# Create the piston
# ---
piston_length = 0.1
piston_radius = 0.08
piston_mass = 0.3
piston_inertia = 0.003

body_piston = chrono.ChBodyEasy()
body_piston.SetMass(piston_mass)
body_piston.SetInertia(piston_inertia)
body_piston.SetPos(chrono.ChVectorD(0, 0.5, 0))
body_piston.SetShapeCylinder(piston_radius, piston_length)
body_piston.SetCollide(True)
system.Add(body_piston)

# ---
# Create the joints
# ---

# Revolute joint for crankshaft to floor
joint_crank_floor = chrono.ChLinkRevolute()
joint_crank_floor.Initialize(body_crank, body_floor, chrono.ChCoordsys(chrono.ChVectorD(0, 0.5, 0)))
system.Add(joint_crank_floor)

# Revolute joint for connecting rod to crankshaft
joint_rod_crank = chrono.ChLinkRevolute()
joint_rod_crank.Initialize(body_rod, body_crank, chrono.ChCoordsys(chrono.ChVectorD(crank_length, 0.5, 0)))
system.Add(joint_rod_crank)

# Revolute joint for connecting rod to piston
joint_rod_piston = chrono.ChLinkRevolute()
joint_rod_piston.Initialize(body_rod, body_piston, chrono.ChCoordsys(chrono.ChVectorD(-rod_length / 2, 0.5, 0)))
system.Add(joint_rod_piston)

#Prismatic joint for piston to floor
joint_piston_floor = chrono.ChLinkPrismatic()
joint_piston_floor.Initialize(body_piston, body_floor, chrono.ChCoordsys(chrono.ChVectorD(0, 0.5, 0)))
joint_piston_floor.SetLimitSpeed(1.0)
joint_piston_floor.SetRestitution(0.0)
system.Add(joint_piston_floor)

# ---
# Add a motor to drive the crankshaft
# ---
motor = chrono.ChFunction_Const()
motor.Set_y(10.0)  # Angular speed in rad/s
motor.Set_min_y(0)
motor.Set_max_y(10.0)

driver = chrono.ChDriver(motor)
joint_crank_floor.Set_motor_function(driver)

# ---
# Visualization
# ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Crank-Slider Mechanism')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()

# ---
# Simulation loop
# ---
time_step = 0.001
time_end = 10.0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > time_end:
        break

vis.Close()