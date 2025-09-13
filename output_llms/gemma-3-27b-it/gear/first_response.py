import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.postprocess as postprocess

# ---
#  Create the simulation system and add items
# ---

# Create a Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# ---
# Create a fixed truss
# ---
body_truss = chrono.ChBodyEasy()
body_truss.SetBodyFixed(True)
body_truss.SetPos(chrono.ChVectorD(0, 0, 0))
body_truss.SetMass(1)
body_truss.AddBox(0.5, 0.5, 0.5)
system.Add(body_truss)

# ---
# Create a rotating bar
# ---
body_bar = chrono.ChBodyEasy()
body_bar.SetMass(1)
body_bar.SetPos(chrono.ChVectorD(0, 1, 0))
body_bar.SetBodyFixed(False)
body_bar.AddCylinder(0.1, 1.0)
body_bar.SetCollide(True)
system.Add(body_bar)

# Add a revolute joint to the bar, connecting it to the truss
joint_bar = chrono.ChLinkRevolute()
joint_bar.Initialize(body_bar, body_truss, chrono.ChCoordsys(chrono.ChVectorD(0, 1, 0)))
system.Add(joint_bar)

# ---
# Create the gears
# ---

# Gear 1 (Driven)
gear1 = chrono.ChBodyEasy()
gear1.SetMass(0.5)
gear1.SetPos(chrono.ChVectorD(1, 1.5, 0))
gear1.SetCollide(True)
gear1.AddCylinder(0.2, 0.2)
system.Add(gear1)

# Gear 2 (Driving) with motor
gear2 = chrono.ChBodyEasy()
gear2.SetMass(0.5)
gear2.SetPos(chrono.ChVectorD(1.5, 2, 0))
gear2.SetCollide(True)
gear2.AddCylinder(0.2, 0.2)
system.Add(gear2)

# Add a revolute joint between the bar and gear 1
joint_gear1 = chrono.ChLinkRevolute()
joint_gear1.Initialize(gear1, body_bar, chrono.ChCoordsys(chrono.ChVectorD(1, 1.5, 0)))
system.Add(joint_gear1)

# Add a revolute joint between gear 1 and gear 2
joint_gear2 = chrono.ChLinkRevolute()
joint_gear2.Initialize(gear2, gear1, chrono.ChCoordsys(chrono.ChVectorD(1.25, 1.75, 0)))
system.Add(joint_gear2)

# Add a ChLinkMotorRotationSpeed to gear 2
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear2, gear2, chrono.ChCoordsys(chrono.ChVectorD(1.5, 2, 0)), 5) # 5 rad/s
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
vis.AddCamera(chrono.ChVectorD(2, 2, -2))
vis.AddTypicalLights()

# Set materials
body_truss.SetMaterialMetallic(0.8, chrono.ChColor(0.5, 0.5, 0.5))
body_bar.SetMaterialMetallic(0.8, chrono.ChColor(0.7, 0.7, 0.7))
gear1.SetMaterialMetallic(0.8, chrono.ChColor(0.6, 0.6, 0.6))
gear2.SetMaterialMetallic(0.8, chrono.ChColor(0.4, 0.4, 0.4))


# ---
# Simulation loop
# ---
time_step = 0.005
time_end = 10

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    system.DoStepDynamics(time_step)

    if system.GetChTime() > time_end:
        break