import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# ---------------------------------------------------------------------
# Initialize the ChronoEngine system

system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# ---------------------------------------------------------------------
# Create ground / truss

ground = chrono.ChBodyEasyBox(0.6, 0.05, 1.0, 1000, True, True)
ground.SetBodyFixed(True)
ground.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(ground)

# Add texture to the ground
ground_texture = chrono.ChTexture()
ground_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
ground.AddAsset(ground_texture)

# ---------------------------------------------------------------------
# Create the crank-slider mechanism

# Dimensions
crank_radius = 0.08
rod_length = 0.25
piston_width = 0.05
piston_height = 0.03
piston_depth = 0.06

# 1. Create the crankshaft
crankshaft = chrono.ChBodyEasyCylinder(0.02, 0.05, 2000, True, True)
crankshaft.SetPos(chrono.ChVectorD(0, 0.1, 0))
crankshaft.SetRot(chrono.Q_from_AngZ(chrono.CH_C_PI_2))
system.Add(crankshaft)

# Add crank pin
crank_pin = chrono.ChBodyEasyCylinder(0.015, 0.03, 2000, True, True)
crank_pin.SetPos(chrono.ChVectorD(crank_radius, 0.1, 0))
system.Add(crank_pin)

# Rigidly connect crank pin to crankshaft
crank_pin_joint = chrono.ChLinkMateFix()
crank_pin_joint.Initialize(crankshaft, crank_pin)
system.Add(crank_pin_joint)

# 2. Create the connecting rod
rod = chrono.ChBodyEasyBox(rod_length, 0.02, 0.02, 1000, True, True)
rod.SetPos(chrono.ChVectorD(crank_radius + rod_length/2, 0.1, 0))
system.Add(rod)

# 3. Create the piston (slider)
piston = chrono.ChBodyEasyBox(piston_width, piston_height, piston_depth, 1000, True, True)
piston.SetPos(chrono.ChVectorD(crank_radius + rod_length, 0.1, 0))
system.Add(piston)

# Add color to elements
col_crank = chrono.ChColorAsset()
col_crank.SetColor(chrono.ChColor(0.2, 0.2, 0.8))
crankshaft.AddAsset(col_crank)
crank_pin.AddAsset(col_crank)

col_rod = chrono.ChColorAsset()
col_rod.SetColor(chrono.ChColor(0.7, 0.2, 0.2))
rod.AddAsset(col_rod)

col_piston = chrono.ChColorAsset()
col_piston.SetColor(chrono.ChColor(0.3, 0.6, 0.3))
piston.AddAsset(col_piston)

# ---------------------------------------------------------------------
# Create the joints

# Revolute joint between ground and crankshaft
joint_crankshaft = chrono.ChLinkLockRevolute()
joint_crankshaft.Initialize(ground, crankshaft, chrono.ChCoordsysD(chrono.ChVectorD(0, 0.1, 0), 
                                                                  chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.Add(joint_crankshaft)

# Revolute joint between connecting rod and crank pin
joint_rod_crank = chrono.ChLinkLockRevolute()
joint_rod_crank.Initialize(rod, crank_pin, chrono.ChCoordsysD(chrono.ChVectorD(crank_radius, 0.1, 0),
                                                             chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.Add(joint_rod_crank)

# Revolute joint between connecting rod and piston
joint_rod_piston = chrono.ChLinkLockRevolute()
joint_rod_piston.Initialize(rod, piston, chrono.ChCoordsysD(chrono.ChVectorD(crank_radius + rod_length, 0.1, 0),
                                                           chrono.Q_from_AngX(chrono.CH_C_PI_2)))
system.Add(joint_rod_piston)

# Prismatic joint between piston and ground (allows only horizontal motion)
joint_piston_ground = chrono.ChLinkLockPrismatic()
joint_piston_ground.Initialize(piston, ground, chrono.ChCoordsysD(chrono.ChVectorD(crank_radius + rod_length, 0.1, 0),
                                                                 chrono.Q_from_AngY(0)))
system.Add(joint_piston_ground)

# ---------------------------------------------------------------------
# Add motor to crankshaft

motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(crankshaft, ground, chrono.ChFrameD(chrono.ChVectorD(0, 0.1, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)))

# Set angular speed (rad/s)
angular_speed = chrono.ChFunction_Const(2.0)
motor.SetSpeedFunction(angular_speed)

system.Add(motor)

# ---------------------------------------------------------------------
# Set up the Irrlicht visualization

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Crank-Slider Mechanism")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.2, 0.3, 0.6), chrono.ChVectorD(0.1, 0.1, 0))
vis.AddTypicalLights()

# ---------------------------------------------------------------------
# Simulation loop

time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(time_step)