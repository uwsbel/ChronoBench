#!/usr/bin/env python3
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.irrlicht as chronoirr

# 1. Initialize Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

# 2. Create ground body
ground = chrono.ChBody()
ground.SetBodyFixed(True)
ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(50, 50, 0.1)
ground.GetCollisionModel().BuildModel()
ground.SetCollide(True)
system.Add(ground)

# 3. Create Turtlebot robot
robot = turtlebot.TurtleBot()
robot.SetDriver(turtlebot.TurtleBotDriver())
initial_pos = chrono.ChVectorD(0, 0, 0.2)
initial_rot = chrono.ChQuaternionD(1, 0, 0, 0)
robot.Initialize(chrono.ChCoordsysD(initial_pos, initial_rot))
robot.AddVisualizationAssets()
robot.AddCollisionShapes()
system.Add(robot.GetChassis())

# 4. Setup Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(2, 1.5, 1), chrono.ChVectorD(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVectorD(5,5,5), chrono.ChVectorD(0,0,0), 10, 2, 10, 40, 512)

# 5. Simulation loop
time_step = 0.01
total_time = 10.0
current_time = 0.0

while vis.Run() and current_time < total_time:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    # Motor control logic
    if current_time < 2.0:
        # Forward motion
        robot.SetMotorSpeeds(1.0, 1.0)
    elif current_time < 4.0:
        # Right turn
        robot.SetMotorSpeeds(1.0, -0.5)
    elif current_time < 6.0:
        # Left turn
        robot.SetMotorSpeeds(-0.5, 1.0)
    elif current_time < 8.0:
        # Backward motion
        robot.SetMotorSpeeds(-1.0, -1.0)
    else:
        # Stop
        robot.SetMotorSpeeds(0, 0)
    
    # Update robot and advance simulation
    robot.Update()
    system.DoStepDynamics(time_step)
    current_time += time_step