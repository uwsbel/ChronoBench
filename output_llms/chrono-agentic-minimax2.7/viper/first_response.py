"""
Viper rover on rigid terrain — mbs_in_scene / robot/rover skill.
System: ChSystemNSC + Bullet collision.
Bodies: fixed ground box + Viper rover (system-owned).
Driver: ViperDCMotorControl (steering-only DC drive).
Motion: steering ramps 0 → ±pi/6 → 0 over 15 s.
"""

import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr

# === Named constants ===
time_step = 1e-3          # Viper/Curiosity standard step
sim_end   = 15.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once
max_steering = math.pi / 6   # ~30 deg, safe for suspension


# === System & gravity (NSC + Bullet) ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground (rigid terrain, NSC material) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))   # top surface at z = -0.5
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Viper rover (system-owned bodies) ===
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# === Visualization — Irrlicht (Initialize FIRST, scene elements AFTER) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain")
vis.Initialize()                                     # FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(
    chrono.ChVector3d(4, -4, 3),                     # eye
    chrono.ChVector3d(0, 0, 0.5),                   # target
)
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.AddGrid(2.0, 2.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop ===
frame = 0
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    t = system.GetChTime()

    # --- steering profile: ramp up → hold → ramp down ---
    if t < 3.0:
        steering = 0.0
    elif 3.0 <= t < 8.0:
        steering = max_steering * (t - 3.0) / 5.0   # ramp up
    elif 8.0 <= t < 13.0:
        steering = max_steering                      # hold at max
    else:
        steering = max_steering * (15.0 - t) / 2.0  # ramp down
    driver.SetSteering(steering)
    rover.Update()

    for _ in range(render_every):
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break

# === Post-processing (review-only) ===
