import pychrono as chrono
import pychrono.irrlicht as chronoirr
import math

# ---
#  Create the simulation system
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# ---
# Create the fixed truss
# ---

# Create a ChBody for the fixed truss
truss = chrono.ChBodyEasy()
truss.SetBodyFixed(True)
truss.SetPos(chrono.ChVectorD(0, 0, 0))
truss.SetMass(1)
truss.AddBox(0.1, 0.1, 0.1)  # Small box for visualization
system.Add(truss)

# ---
# Create the rotating bar
# ---

# Create a ChBody for the rotating bar
bar = chrono.ChBodyEasy()
bar.SetMass(1)
bar.SetPos(chrono.ChVectorD(0, 0.1, 0))
bar.SetBodyFixed(False)
bar.AddCylinder(0.2, 0.05)  # Cylinder for the bar
system.Add(bar)

# Create a ChLinkRevolute to connect the bar to the truss
revolute_bar = chrono.ChLinkRevolute()
revolute_bar.Initialize(truss, bar, chrono.ChCoordsys(chrono.ChVectorD(0, 0, 0)))
system.Add(revolute_bar)


# ---
# Create the gears
# ---

# Gear 1 parameters
gear1_radius = 0.1
gear1_center = chrono.ChVectorD(0.2, 0.1, 0)
gear1_thickness = 0.05

# Gear 2 parameters
gear2_radius = 0.2
gear2_center = chrono.ChVectorD(0.4, 0.1, 0)
gear2_thickness = 0.05

# Create Gear 1
gear1 = chrono.ChBodyEasy()
gear1.SetMass(1)
gear1.SetPos(gear1_center)
gear1.SetBodyFixed(False)
gear1.AddCylinder(gear1_radius, gear1_thickness)
system.Add(gear1)

# Create Gear 2
gear2 = chrono.ChBodyEasy()
gear2.SetMass(1)
gear2.SetPos(gear2_center)
gear2.SetBodyFixed(False)
gear2.AddCylinder(gear2_radius, gear2_thickness)
system.Add(gear2)

# Create a ChLinkMateRevolute to connect the gears
link_gear = chrono.ChLinkMateRevolute()
link_gear.Initialize(gear1, gear2, chrono.ChCoordsys(gear1_center + chrono.ChVectorD(gear1_radius, 0, 0)))
system.Add(link_gear)

# ---
# Create a gear motor for Gear 1
# ---

# Create a ChLinkMotorRotationSpeed to drive Gear 1
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(truss, gear1, chrono.ChCoordsys(gear1_center))
motor.Set_Rot_speed(2)  # Set the rotation speed in rad/s
motor.Set_Torque(10)
system.Add(motor)

# ---
# Visualization
# ---

# Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(2, 1.5, -2))
vis.AddTypicalLights()

# Set material properties for better visualization
truss.SetMaterialMetallic(0.8, chrono.ChColor(0.5, 0.5, 0.5))
bar.SetMaterialMetallic(0.8, chrono.ChColor(0.7, 0.7, 0.7))
gear1.SetMaterialMetallic(0.8, chrono.ChColor(0.3, 0.6, 0.9))
gear2.SetMaterialMetallic(0.8, chrono.ChColor(0.3, 0.6, 0.9))

# ---
# Simulation loop
# ---

# Simulation time step
time_step = 0.001

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)