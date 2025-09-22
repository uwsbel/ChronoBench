import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set this to your Chrono data directory

# Create the Chrono physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add the required physical systems and objects

# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the Turtlebot
turtlebot = chrono.ChBodyEasyBox(0.3, 0.1, 0.3, 10, True, True)
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # Default orientation
system.Add(turtlebot)

# Create wheels (simplified as cylinders)
wheel_radius = 0.05
wheel_width = 0.02

# Left wheel
left_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1, True, True)
left_wheel.SetPos(chrono.ChVectorD(0.1, 0.1, 0.15))
system.Add(left_wheel)

# Right wheel
right_wheel = chrono.ChBodyEasyCylinder(wheel_radius, wheel_width, 1, True, True)
right_wheel.SetPos(chrono.ChVectorD(0.1, 0.1, -0.15))
system.Add(right_wheel)

# Create motor between wheels and chassis
link_motor_L = chrono.ChLinkMotorRotationSpeed()
link_motor_L.Initialize(turtlebot, left_wheel, chrono.ChFrameD(chrono.ChVectorD(0.1, 0.1, 0.15)))
system.Add(link_motor_L)

link_motor_R = chrono.ChLinkMotorRotationSpeed()
link_motor_R.Initialize(turtlebot, right_wheel, chrono.ChFrameD(chrono.ChVectorD(0.1, 0.1, -0.15)))
system.Add(link_motor_R)

# 3. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.SetCameraVertical(chrono.ChVectorD(0, 1, 0))

# Simulation parameters
time_step = 0.01
sim_time = 10.0
current_time = 0.0

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Apply motor controls at specific times
    if current_time < 2.0:
        # Move forward
        link_motor_L.SetMotorSpeed(10.0)
        link_motor_R.SetMotorSpeed(10.0)
    elif current_time < 4.0:
        # Turn left
        link_motor_L.SetMotorSpeed(-5.0)
        link_motor_R.SetMotorSpeed(5.0)
    elif current_time < 6.0:
        # Move forward
        link_motor_L.SetMotorSpeed(10.0)
        link_motor_R.SetMotorSpeed(10.0)
    elif current_time < 8.0:
        # Turn right
        link_motor_L.SetMotorSpeed(5.0)
        link_motor_R.SetMotorSpeed(-5.0)
    else:
        # Stop
        link_motor_L.SetMotorSpeed(0.0)
        link_motor_R.SetMotorSpeed(0.0)

    # Advance simulation
    system.DoStepDynamics(time_step)
    current_time += time_step

    if current_time >= sim_time:
        break

    vis.DoStepDynamics(time_step)