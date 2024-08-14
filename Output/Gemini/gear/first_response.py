import pychrono as chrono
import pychrono.irrlicht as chronoirr

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create the Chrono system
system = chrono.ChSystemNSC()

# Set gravitational acceleration
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the Irrlicht visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Epicyclic Gears')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 1, 3))
vis.AddTypicalLights()

# Create a fixed truss (e.g., a ground plane)
ground = chrono.ChBodyEasyBox(1, 0.1, 1, 1000)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, -0.05, 0))
system.Add(ground)

# Create a rotating bar
bar_length = 0.5
bar = chrono.ChBodyEasyCylinder(bar_length/2, 0.02, 1000)
bar.SetPos(chrono.ChVectorD(0, 0.2, 0))
system.Add(bar)

# Create the gears
gear1_radius = 0.1
gear2_radius = 0.2
gear1 = chrono.ChBodyEasyCylinder(gear1_radius, 0.03, 1000)
gear1.SetPos(chrono.ChVectorD(0, 0.2, 0))
system.Add(gear1)
gear2 = chrono.ChBodyEasyCylinder(gear2_radius, 0.03, 1000)
gear2.SetPos(chrono.ChVectorD(gear1_radius + gear2_radius, 0.2, 0))
system.Add(gear2)

# Create revolute joints between the bar and gears
revolute_joint1 = chrono.ChLinkRevolute()
revolute_joint1.Initialize(bar, gear1, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.2, 0), chrono.QUNIT))
system.Add(revolute_joint1)

revolute_joint2 = chrono.ChLinkRevolute()
revolute_joint2.Initialize(bar, gear2, chrono.ChCoordsysD(chrono.ChVectorD(gear1_radius + gear2_radius, 0.2, 0), chrono.QUNIT))
system.Add(revolute_joint2)

# Create a gear motor to enforce a constant rotation speed on gear1
motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(gear1, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0.2, 0)))
motor.SetMotorFunction(chrono.ChFunction_Const(2 * chrono.CH_C_PI))  # 1 revolution per second
system.Add(motor)

# Set materials and colors for visualization
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/concrete.jpg'))
bar.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/aluminum.jpg'))
gear1.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/steel.jpg'))
gear2.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile('textures/steel.jpg'))

# ---------------------------------------------------------------------
#
#  Simulate the system
#

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(0.01)