import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as turtlebot
from pychrono import irrlicht as chronoirr

# -------------------------------------------------------------
# 1.  Create Chrono system
# -------------------------------------------------------------
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

# Default collision envelope / margin (must be set before bodies are created)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# -------------------------------------------------------------
# 2.  Ground body – terrain plane
# -------------------------------------------------------------
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))          # <-- adjusted to -0.6
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# -------------------------------------------------------------
# 3.  Turtlebot robot
# -------------------------------------------------------------
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
robot = turtlebot.TurtleBot(system, init_pos, init_rot)
robot.Initialize()

# -------------------------------------------------------------
# 4.  Runtime visualisation (Irrlicht)
# -------------------------------------------------------------
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Turtlebot Robot - Rigid terrain')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512)

# (Shadow line removed – no longer commented‑out)

# -------------------------------------------------------------
# 5.  Helper – movement controller
# -------------------------------------------------------------
LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1

def move(mode: str) -> None:
    """Set wheel motor speeds according to the requested *mode*.

    Parameters
    ----------
    mode : str
        One of ``'straight'``, ``'left'``, ``'right'``.

    Raises
    ------
    ValueError
        If *mode* is not recognised.
    """
    if mode == "straight":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == "left":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  0)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, math.pi)
    elif mode == "right":
        robot.SetMotorSpeed(LEFT_DRIVE_WHEEL,  math.pi)
        robot.SetMotorSpeed(RIGHT_DRIVE_WHEEL, 0)
    else:
        raise ValueError(f"Invalid movement mode: '{mode}'")

# -------------------------------------------------------------
# 6.  Simulation loop
# -------------------------------------------------------------
time_step = 2e-3          # seconds
time = 0.0
current_mode = None

while vis.Run():
    # Decide which motion to apply based on elapsed time
    if time < 5.0:
        desired_mode = "straight"
    elif time < 10.0:
        desired_mode = "left"
    else:
        desired_mode = "right"

    # Apply the new mode only when it changes (avoids redundant calls)
    if desired_mode != current_mode:
        print(f"[{time:6.2f} s] Changing movement to '{desired_mode}'")
        move(desired_mode)
        current_mode = desired_mode

    # Advance simulation time
    time += time_step

    # Render the scene
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Integrate the dynamics one step
    system.DoStepDynamics(time_step)