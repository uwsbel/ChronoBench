import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# 1. Initialize the PyChrono environment and core components
chrono.SetChronoDataPath("PATH_TO_CHRONO_DATA")  # Set your Chrono data path here

# Create the physical system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  # Set gravity

# 2. Add physical systems and objects
# Create the ground body
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

# Create the Turtlebot
turtlebot = chrono.ChBody()
system.Add(turtlebot)

# Set initial position and orientation
turtlebot.SetPos(chrono.ChVectorD(0, 0.1, 0))
turtlebot.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))  # No initial rotation

# Create a simple visualization shape for the Turtlebot
turtlebot_visual = chrono.ChBoxShape()
turtlebot_visual.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 0.1, 0.2))
turtlebot.AddVisualShape(turtlebot_visual)

# Create wheels (simplified representation)
wheel_radius = 0.05
wheel_width = 0.02

# Left wheel
left_wheel = chrono.ChBody()
left_wheel.SetPos(chrono.ChVectorD(-0.1, 0.05, 0))
system.Add(left_wheel)
left_wheel_visual = chrono.ChCylinderShape()
left_wheel_visual.GetCylinderGeometry().rad = wheel_radius
left_wheel_visual.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -wheel_width/2)
left_wheel_visual.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, wheel_width/2)
left_wheel.AddVisualShape(left_wheel_visual)

# Right wheel
right_wheel = chrono.ChBody()
right_wheel.SetPos(chrono.ChVectorD(0.1, 0.05, 0))
system.Add(right_wheel)
right_wheel_visual = chrono.ChCylinderShape()
right_wheel_visual.GetCylinderGeometry().rad = wheel_radius
right_wheel_visual.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -wheel_width/2)
right_wheel_visual.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, wheel_width/2)
right_wheel.AddVisualShape(right_wheel_visual)

# Create joints to connect wheels to the body
# Left wheel joint (revolute joint)
left_joint = chrono.ChLinkLockRevolute()
left_joint.Initialize(turtlebot, left_wheel, chrono.ChCoordsysD(chrono.ChVectorD(-0.1, 0.05, 0), chrono.QUNIT))
system.Add(left_joint)

# Right wheel joint (revolute joint)
right_joint = chrono.ChLinkLockRevolute()
right_joint.Initialize(turtlebot, right_wheel, chrono.ChCoordsysD(chrono.ChVectorD(0.1, 0.05, 0), chrono.QUNIT))
system.Add(right_joint)

# 3. Set up visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5, 0.5, 1.5), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(1, 2, 1), chrono.ChVectorD(0, 0, 0), 3, 1, 10, 40, 512)

# 4. Simulation loop
time_step = 0.01
sim_time = 0
max_sim_time = 10

while vis.Run() and sim_time < max_sim_time:
    # Update simulation time
    sim_time += time_step

    # Motor control logic
    if sim_time < 2:
        # Turn left (left wheel forward, right wheel backward)
        left_wheel.SetWvel_loc(chrono.ChVectorD(0, 5, 0))  # 5 rad/s forward
        right_wheel.SetWvel_loc(chrono.ChVectorD(0, -5, 0))  # 5 rad/s backward
    elif sim_time < 4:
        # Move forward
        left_wheel.SetWvel_loc(chrono.ChVectorD(0, 5, 0))
        right_wheel.SetWvel_loc(chrono.ChVectorD(0, 5, 0))
    elif sim_time < 6:
        # Turn right (left wheel backward, right wheel forward)
        left_wheel.SetWvel_loc(chrono.ChVectorD(0, -5, 0))
        right_wheel.SetWvel_loc(chrono.ChVectorD(0, 5, 0))
    else:
        # Stop
        left_wheel.SetWvel_loc(chrono.ChVectorD(0, 0, 0))
        right_wheel.SetWvel_loc(chrono.ChVectorD(0, 0, 0))

    # Perform the simulation step
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance simulation
    system.DoStepDynamics(time_step)

# Print final position
print("Final position:", turtlebot.GetPos())