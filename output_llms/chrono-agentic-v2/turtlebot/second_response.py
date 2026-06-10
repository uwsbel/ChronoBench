"""
TurtleBot differential-drive simulation on rigid terrain (ChSystemNSC).

Models a TurtleBot robot performing a programmed motion sequence:
- Straight driving for the first 5 seconds.
- Left turn for the next 5 seconds (right wheel drives, left wheel stopped).
- Right turn for all time thereafter (left wheel drives, right wheel stopped).

The move(mode) function dispatches motor speed commands to the two drive
wheels based on the requested mode. Expected behavior: the robot translates
forward, pivots left, then pivots right, all visible in the Irrlicht window.
"""

import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
TIME_STEP    = 2e-3     # TurtleBot canonical timestep (s)
SIM_END      = 20.0     # total simulation duration (s)
RENDER_FPS   = 50.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

WHEEL_FWD  = -math.pi  # rad/s — forward motion on each drive wheel
WHEEL_STOP = 0.0       # rad/s — wheel stopped

LEFT_WHEEL  = robot.LD  # WheelID: 0 = left drive wheel
RIGHT_WHEEL = robot.RD  # WheelID: 1 = right drive wheel

GROUND_Z    = -0.6  # ground body center Z; top surface at z = -0.1
ROBOT_SPAWN = chrono.ChVector3d(0, 0, 0.2)   # initial robot position
ROBOT_ROT   = chrono.ChQuaterniond(1, 0, 0, 0)  # identity rotation (w,x,y,z)


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Ground body ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === TurtleBot robot ===
robot_tb = robot.TurtleBot(system, ROBOT_SPAWN, ROBOT_ROT)
robot_tb.Initialize()


# === Movement control function ===
def move(mode):
    """Set TurtleBot drive-wheel speeds for the requested motion mode.

    Args:
        mode (str): Motion mode — 'straight', 'left', or 'right'.

    Raises:
        ValueError: if mode is not one of the three accepted strings.
    """
    if mode == "straight":
        robot_tb.SetMotorSpeed(WHEEL_FWD,  LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_FWD,  RIGHT_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(WHEEL_STOP, LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_FWD,  RIGHT_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(WHEEL_FWD,  LEFT_WHEEL)
        robot_tb.SetMotorSpeed(WHEEL_STOP, RIGHT_WHEEL)
    else:
        raise ValueError(
            f"Invalid move mode '{mode}'. Expected 'straight', 'left', or 'right'."
        )


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Differential Drive Motion Sequence")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3.0, 2.0), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)


# === Main loop ===
frame = 0
_prev_mode = None  # cache: last mode — avoids redundant print calls

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        sim_time = system.GetChTime()  # cache: fetched once per frame, reused below

        # Determine motion mode based on elapsed time
        if sim_time < 5.0:
            current_mode = "straight"
        elif sim_time < 10.0:
            current_mode = "left"
        else:
            current_mode = "right"

        move(current_mode)

        # Print action announcement on mode transitions
        if current_mode != _prev_mode:
            if current_mode == "straight":
                print(f"[t={sim_time:.3f}s] Moving straight")
            elif current_mode == "left":
                print(f"[t={sim_time:.3f}s] Turning left")
            else:
                print(f"[t={sim_time:.3f}s] Turning right")
            _prev_mode = current_mode

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence or bad move mode
    import traceback
    traceback.print_exc()
    raise
