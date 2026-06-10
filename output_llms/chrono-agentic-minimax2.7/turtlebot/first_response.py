"""
TurtleBot differential-drive robot on rigid terrain.
System: ChSystemNSC (non-smooth contact, Z-down).
Bodies: fixed ground box + TurtleBot rover (own bodies).
Behavior: robot drives forward, then executes left pivot turn,
          then right pivot turn at specified simulation times.
"""

import math
import os

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr


# === System & gravity ===
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1.0, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1.0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === TurtleBot ===
init_pos = chrono.ChVector3d(0, 0, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity
turtlebot = robot.TurtleBot(sys, init_pos, init_rot)
turtlebot.Initialize()

LEFT_DRIVE_WHEEL = 0
RIGHT_DRIVE_WHEEL = 1
time_step = 2e-3  # TurtleBot default

# === Visualization (Irrlicht) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Rigid Terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(3, -3, 2), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, -0.5), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only recording scaffolding ===
sim_end = 20.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))


# === Main loop ===
while vis.Run() and sys.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


    for _ in range(render_every):
        t = sys.GetChTime()

        # Motor control schedule
        if 0.998 < t < 1.002:
            # forward drive at speed 4 rad/s
            turtlebot.SetMotorSpeed(4.0, LEFT_DRIVE_WHEEL)
            turtlebot.SetMotorSpeed(4.0, RIGHT_DRIVE_WHEEL)
        if 4.998 < t < 5.002:
            # left pivot: stop left, spin right
            turtlebot.SetMotorSpeed(0.0, LEFT_DRIVE_WHEEL)
            turtlebot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
        if 7.998 < t < 8.002:
            # right pivot: spin left, stop right
            turtlebot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
            turtlebot.SetMotorSpeed(0.0, RIGHT_DRIVE_WHEEL)
        if 10.998 < t < 11.002:
            # resume forward
            turtlebot.SetMotorSpeed(4.0, LEFT_DRIVE_WHEEL)
            turtlebot.SetMotorSpeed(4.0, RIGHT_DRIVE_WHEEL)

        sys.DoStepDynamics(time_step)
        if sys.GetChTime() >= sim_end:
            break
