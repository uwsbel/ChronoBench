importchrono
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Initialize the Chrono system
chrono.ChSystem.SetRootPath("chrono_data/")
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

# Create the ground
ground = chrono.ChBody()
ground.SetPos(chrono.ChVectorD(0, 0, 0))
ground.SetMass(0)  # Fixed body
ground.GetCollisionModel().AddBox(chrono.ChCollisionBox(chrono.ChVectorD(10, 1, 10)))
ground_mat = chrono.ChMaterialSurface()
ground_mat.SetFriction(0.5)
ground_mat.SetRestitution(0.2)
ground.GetCollisionModel().SetMaterial(ground_mat)
system.Add(ground)

# Create the rover body
rover = chrono.ChBody()
rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
rover.SetMass(200)
rover.GetCollisionModel().AddBox(chrono.ChCollisionBox(chrono.ChVectorD(0.5, 0.5, 0.5)))
system.Add(rover)

# Add wheels
wheel_pos = [(-0.5, -0.25, 0.5), (-0.5, -0.25, -0.5), (0.5, -0.25, 0.5), (0.5, -0.25, -0.5)]
for pos in wheel_pos:
    wheel = chrono.ChBody()
    wheel.SetPos(chrono.ChVectorD(*pos))
    wheel.SetMass(10)
    wheel.GetCollisionModel().AddCylinder(chrono.ChCollisionCylinder(0.2, 0.1))
    system.Add(wheel)
    
    # Add wheel to body joint
    joint = chrono.ChLinkRev()
    joint.Initialize(rover, wheel, chrono.ChCoordsysD(chrono.ChVectorD(*pos), chrono.ChQuaternionD(1,0,0,0)))
    system.Add(joint)
    
    # Add motor control
    motor = chrono.ChMotorRotation()
    motor.SetMode(chrono.ChMotorRotation::MODE_SPEED)
    motor.SetSpeedTarget(0)
    motor.Initialize(joint.Get ShaftA(), joint.Get ShaftB(), chrono.ChVectorD(0,1,0))
    system.Add(motor)

# Add driver for real-time control
driver = chrono.ChDriver()
driver.Attach(rover)
system.Add(driver)

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowTitle("Curiosity Rover Simulation")
vis.SetCamera(chronoirr.ChCamera(60, 1.2, 100))
vis.SetLightDirection(chrono.ChVectorD(0, -1, 0))
vis.SetLightIntensity(1)
vis.SetLightPosition(chrono.ChVectorD(0, 10, 0))
vis.AddSky()
vis.AddLogo()

# Add textures and visual assets
ground_texture = chronoirr.ChTexture()
ground_texture.SetTextureFilename("textures/concrete.jpg")
ground.GetAssets().push_back(ground_texture)

rover_texture = chronoirr.ChTexture()
rover_texture.SetTextureFilename("textures/metal.jpg")
rover.GetAssets().push_back(rover_texture)

# Add logo billboard
logo = chronoirr.ChBillboard()
logo.SetPosition(chrono.ChVectorD(0, 0.5, 0))
logo.SetSize(0.2, 0.1)
logo.SetTextureFilename("textures/logo.png")
vis.AddBillboard(logo)

# Simulation loop
while vis.Run():
    # Handle user input
    if vis.KeyPress(chrono.CH_KEY_UP):
        driver.GetSteering().SetSpeed(1)
    elif vis.KeyPress(chrono.CH_KEY_DOWN):
        driver.GetSteering().SetSpeed(-1)
    elif vis.KeyPress(chrono.CH_KEY_LEFT):
        driver.GetSteering().SetSteering(-0.5)
    elif vis.KeyPress(chrono.CH_KEY_RIGHT):
        driver.GetSteering().SetSteering(0.5)
    
    # Update simulation
    system.DoStepDynamics(0.001)
    vis.Render()