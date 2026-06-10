"""
TurtleBot differential-drive simulation on rigid terrain.

System: ChSystemNSC with Bullet collision.
Bodies: fixed ground box (concrete texture), TurtleBot rover (built-in robot model).
Behavior: drives straight for the first 5 s, turns left for the next 5 s,
then turns right for the remainder of the simulation.
The move() function controls wheel motor speeds to achieve each mode.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Constants ===
TIME_STEP   = 2e-3        # TurtleBot standard time step (s)
SIM_END     = 20.0        # total simulation duration (s)
RENDER_FPS  = 50.0
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

LEFT_WHEEL  = 0           # TurtleBot wheel IDs
RIGHT_WHEEL = 1

# Motor speeds (rad/s)
SPEED_FWD   = math.pi     # forward (both wheels same magnitude)
SPEED_TURN  = math.pi     # differential-drive turn speed

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

# Ground box: 20×20×1 m, top surface at z = -0.1 (box center at z=-0.6)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)

# === TurtleBot robot ===
init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()                           # no-arg; pose set in constructor

# === Movement controller ===
def move(mode: str) -> None:
    """Set TurtleBot wheel motor speeds for the given movement mode.

    Parameters
    ----------
    mode : str
        'straight' – both wheels forward at SPEED_FWD
        'left'     – left wheel stopped, right wheel forward (pivot left)
        'right'    – right wheel stopped, left wheel forward (pivot right)

    Raises
    ------
    ValueError
        If mode is not one of the three recognized values.
    """
    if mode == "straight":
        robot_tb.SetMotorSpeed(-SPEED_FWD,  LEFT_WHEEL)
        robot_tb.SetMotorSpeed(-SPEED_FWD,  RIGHT_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0,           LEFT_WHEEL)
        robot_tb.SetMotorSpeed(-SPEED_TURN, RIGHT_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(-SPEED_TURN, LEFT_WHEEL)
        robot_tb.SetMotorSpeed(0,           RIGHT_WHEEL)
    else:
        raise ValueError(f"Unknown move mode: '{mode}'. "
                         "Expected 'straight', 'left', or 'right'.")


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Differential Drive")
vis.Initialize()                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -3.0, 2.0), chrono.ChVector3d(0, 0, 0.2))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512
)

# === Review-only setup ===


# === Main loop ===
current_mode = "straight"
move("straight")  # initial motor command

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            t = system.GetChTime()

            # Determine movement mode from current simulation time
            if t < 5.0:
                new_mode = "straight"
            elif t < 10.0:
                new_mode = "left"
            else:
                new_mode = "right"

            # Update motor speeds only when mode changes (avoid redundant calls)
            if new_mode != current_mode:
                if new_mode == "straight":
                    print(f"[t={t:.2f}s] Moving straight")
                elif new_mode == "left":
                    print(f"[t={t:.2f}s] Turning left")
                else:
                    print(f"[t={t:.2f}s] Turning right")
                move(new_mode)
                current_mode = new_mode


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad motor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
