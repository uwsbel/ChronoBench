import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

# -----------------------------------------------------------------------------
# 1) Create Chrono system
# -----------------------------------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
# gravity downwards in Z
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# -----------------------------------------------------------------------------
# 2) Create ground plane
# -----------------------------------------------------------------------------
ground_mat = chrono.ChContactMaterialNSC()
# NOTE: moved from z=-1 to z=-0.6 per spec
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# -----------------------------------------------------------------------------
# 3) Create Turtlebot robot
# -----------------------------------------------------------------------------
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# wheel‐ID constants
LEFT_DRIVE_WHEEL  = 0
RIGHT_DRIVE_WHEEL = 1

# -----------------------------------------------------------------------------
# 4) Visualization setup
# -----------------------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2),
              chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5),
                       chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)
# (Removed the commented-out EnableShadows line per instructions)

# -----------------------------------------------------------------------------
# 5) Movement helper
# -----------------------------------------------------------------------------
def move(mode):
    """Set motor speeds for the given mode: 'straight', 'left', or 'right'."""
    if mode == 'straight':
        # both wheels forward
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':
        # left wheel stopped, right wheel forward
        robot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':
        # left wheel forward, right wheel stopped
        robot.SetMotorSpeed(math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid move mode '{mode}'")

# -----------------------------------------------------------------------------
# 6) Simulation loop
# -----------------------------------------------------------------------------
time_step = 2e-3
time = 0.0
prev_mode = None

while vis.Run():
    # Decide what mode we are in
    if time < 5.0:
        mode = 'straight'
    elif time < 10.0:
        mode = 'left'
    else:
        mode = 'right'

    # Print only when the mode changes
    if mode != prev_mode:
        print(f"[t = {time:.2f} s] Switching to '{mode}' motion")
        prev_mode = mode

    # Apply the motion
    move(mode)

    # Render
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Advance time simulation
    system.DoStepDynamics(time_step)
    time += time_step