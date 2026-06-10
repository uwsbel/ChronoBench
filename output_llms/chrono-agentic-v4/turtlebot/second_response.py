"""
TurtleBot differential-drive robot simulation on rigid terrain.
System type: ChSystemNSC (non-smooth contact, Bullet collision).
The robot drives straight, then turns left, then turns right.
"""
import math
import os

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))   # top surface at z=-0.1
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === TurtleBot ===
init_pos = chrono.ChVector3d(0, 0.2, -0.1)    # on ground surface
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)    # identity
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()

LEFT_DRIVE_WHEEL = 0   # WheelID: 0 = LEFT
RIGHT_DRIVE_WHEEL = 1  # WheelID: 1 = RIGHT

# Motor speed constants (rad/s)
STRAIGHT_SPEED = -10.0  # rad/s for both wheels (negative = forward)
TURN_SPEED = 0.0        # rad/s for pivot turn


def move(mode):
    """Set TurtleBot wheel motor speeds based on movement mode.

    Modes:
        straight : both wheels at STRAIGHT_SPEED (forward)
        left     : left wheel at TURN_SPEED, right wheel at STRAIGHT_SPEED (pivot left)
        right    : left wheel at STRAIGHT_SPEED, right wheel at TURN_SPEED (pivot right)
    """
    if mode == "straight":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(TURN_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(STRAIGHT_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(TURN_SPEED, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError(f"Invalid move mode: {mode!r}. Expected 'straight', 'left', or 'right'.")


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Differential Drive")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 20, 20,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.1), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Simulation loop ===
time_step = 2e-3        # TurtleBot recommended timestep
sim_end = 15.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

os.makedirs("frames", exist_ok=True)   # guard against missing output dir

REC = bool(os.environ.get("SIMBENCH_RECORD"))

frame = 0
sim_time = 0.0

try:
    while vis.Run() and sim_time < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        if REC:
            frame += 1

        for _ in range(render_every):
            # Determine movement mode based on elapsed time
            if sim_time < 5.0:
                mode = "straight"
            elif sim_time < 10.0:
                mode = "left"
            else:
                mode = "right"

            move(mode)  # call every step - TurtleBot needs motor speed set each step
            if int(sim_time * 10) != int((sim_time - time_step) * 10):
                print(f"[t={sim_time:.2f}s] mode: {mode}")

            system.DoStepDynamics(time_step)
            sim_time = system.GetChTime()
            if sim_time >= sim_end:
                break

except (RuntimeError, ValueError) as exc:
    import traceback
    traceback.print_exc()
    raise
finally:
    pass   # no review-only writers to flush

# === Post-processing ===
