import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath('PATH_TO_CHRONO_DATA')  # Set this to your Chrono data directory

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects

# Create the ground
ground = chrono.ChBodyEasyBox(10, 0.5, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
ground.SetBodyFixed(True)
ground.GetVisualModel().SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Create the rover (simplified model with 6 wheels)
rover = chrono.ChBodyEasyBox(1.5, 0.5, 1.0, 500, True, True)
rover.SetPos(chrono.ChVectorD(0, 0.5, 0))
system.Add(rover)

# Create wheels (6 wheels for the rover)
wheel_radius = 0.25
wheel_width = 0.2
wheel_positions = [
    chrono.ChVectorD(0.8, 0.3, 0.5),  # Front right
    chrono.ChVectorD(0.8, 0.3, -0.5), # Front left
    chrono.ChVectorD(0, 0.3, 0.5),    # Middle right
    chrono.ChVectorD(0, 0.3, -0.5),   # Middle left
    chrono.ChVectorD(-0.8, 0.3, 0.5), # Rear right
    chrono.ChVectorD(-0.8, 0.3, -0.5) # Rear left
]

wheels = []
for i, pos in enumerate(wheel_positions):
    wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 10, True, True)
    wheel.SetPos(pos)
    wheel.SetRot(chrono.Q_from_AngAxis(chrono.CH_PI/2, chrono.ChVectorD(0,0,1)))
    system.Add(wheel)
    wheels.append(wheel)

    # Connect wheel to rover with a revolute joint
    joint = chrono.ChLinkLockRevolute()
    joint.Initialize(wheel, rover, chrono.ChCoordsysD(pos))
    system.Add(joint)

# Create motor control for wheels
motor = chrono.ChLinkMotorRotationTorque()
motor.Initialize(wheels[0], rover, chrono.ChFrameD(wheel_positions[0]))
system.Add(motor)
motor.SetTorqueFunction(chrono.ChFunction_Const(5))  # Constant torque for simplicity

# 3. Set necessary default parameters

# Set collision properties
for body in [ground, rover] + wheels:
    body.GetCollisionModel().SetFamily(1)
    body.GetCollisionModel().SetFamilyMaskNoCollisionWithFamily(1)

# 4. Visualization setup
application = chronoirr.ChIrrApp(system, "Curiosity Rover Simulation", chronoirr.dimension2du(1280, 720))
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights(chrono.ChVectorD(30, 50, 30), chrono.ChVectorD(0, 0, 0))
application.AddTypicalCamera(chrono.ChVectorD(5, 2, 0))
application.AssetBindAll()
application.AssetUpdateAll()

# Customize camera settings
camera = application.GetDevice().getSceneManager().addCameraSceneNode()
camera.setPosition(chrono.ChVectorD(5, 5, 5))
camera.setTarget(chrono.ChVectorD(0, 0, 0))

# Enable shadows
application.AddShadowAll()

# Simulation loop
application.SetTimestep(0.01)
while(application.GetDevice().run()):
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

    # Simple steering control (rotate wheels based on time)
    time = system.GetChTime()
    for i, wheel in enumerate(wheels):
        if i % 2 == 0:  # Right side wheels
            wheel.SetWvel_loc(chrono.ChVectorD(0, 10*np.sin(time), 0))
        else:  # Left side wheels
            wheel.SetWvel_loc(chrono.ChVectorD(0, -10*np.sin(time), 0))