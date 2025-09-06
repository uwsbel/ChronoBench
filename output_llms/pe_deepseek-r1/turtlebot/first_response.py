import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as chronorobo
import pychrono.irrlicht as chronoirr

# ========================
# 1. System Initialization
# ========================
system = chrono.ChSystemSMC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# ===================
# 2. Terrain Creation
# ===================
ground = chrono.ChBody()
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, 0, -0.1))
ground.EnableCollision(True)

# Create visual and collision shapes
ground_box = chrono.ChVisualShapeBox(20, 20, 0.2)
ground_box.SetColor(chrono.ChColor(0.2, 0.3, 0.1))
ground.AddVisualShape(ground_box)

ground_coll = chrono.ChCollisionShapeBox(20, 20, 0.2)
ground.AddCollisionShape(ground_coll)

system.Add(ground)

# =====================
# 3. Turtlebot Creation
# =====================
# Initialize Turtlebot at position (0,0,0) with 20° rotation around Z
init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.QuatFromAngleZ(20 * chrono.CH_DEG_TO_RAD)
turtlebot = chronorobo.TurtleBot()
turtlebot.Initialize(chrono.ChCoordsysd(init_pos, init_rot), system)

# Set collision and visualization properties
turtlebot.SetRobotVisualizationType(chronorobo.RobotVisualizationType_MESH)
turtlebot.SetWheelVisualizationType(chronorobo.RobotVisualizationType_PRIMITIVES)

# ========================
# 4. Visualization Setup
# ========================
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Turtlebot Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + "logo_pychrono_alpha.png")
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1, 2, 1.5))
vis.AddTypicalLights()

# =========================
# 5. Simulation Parameters
# =========================
time_step = 0.005
sim_time = 0
max_time = 30.0

# Motor control parameters
motor_speed = 3.0  # rad/s
turn_duration = 2.0  # seconds
current_motor_speed = 0

# ======================
# 6. Simulation Loop
# ======================
while vis.Run() and sim_time < max_time:
    # ----------------------
    # Motor Control Logic
    # ----------------------
    if sim_time < 5.0:
        # Forward motion for first 5 seconds
        left_speed = motor_speed
        right_speed = motor_speed
    elif sim_time < 5.0 + turn_duration:
        # Turn right for 2 seconds
        left_speed = motor_speed
        right_speed = -motor_speed
    elif sim_time < 10.0 + turn_duration:
        # Forward again
        left_speed = motor_speed
        right_speed = motor_speed
    elif sim_time < 10.0 + 2*turn_duration:
        # Turn left for 2 seconds
        left_speed = -motor_speed
        right_speed = motor_speed
    else:
        # Continue forward
        left_speed = motor_speed
        right_speed = motor_speed

    # Apply motor speeds
    turtlebot.SetMotorSpeeds(left_speed, right_speed)
    
    # ----------------------
    # Simulation Step
    # ----------------------
    vis.BeginScene()
    vis.Render()
    
    # Draw robot trajectory
    trail_color = chrono.ChColor(1, 0, 0)
    vis.DrawLine(turtlebot.GetChassis().GetPos(), 
                 turtlebot.GetChassis().GetPos() + chrono.ChVector3d(0.1,0,0),
                 trail_color)
    
    # Display information
    vis.GetGUI().SetStaticText(f"Time: {sim_time:.2f}s\n" 
                               f"Left Motor: {left_speed:.2f} rad/s\n"
                               f"Right Motor: {right_speed:.2f} rad/s", 
                               400, 10)
    
    vis.EndScene()
    system.DoStepDynamics(time_step)
    sim_time += time_step

print("Simulation completed successfully")