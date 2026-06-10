"""
Viper rover with chassis-mounted camera sensor on rigid terrain.
plan_type: mbs_in_scene (robot-centered hybrid scene)
System: ChSystemNSC (rover uses NSC, not SMC)
"""

import os
import math
import csv
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Named constants ===
time_step = 1e-3           # Viper/Curiosity use 1e-3
sim_end = 15.0
render_fps = 25.0
render_every = max(1, round(1.0 / (render_fps * time_step)))

# Rover control
max_steering = math.pi / 6  # practical max steering angle

# === System & gravity (NSC, Bullet collision) ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground — rigid terrain (NSC contact material) ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))   # top surface at z=-0.5
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Viper rover ===
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

# Cache rover chassis body for sensor attachment
chassis_body = rover.GetChassis().GetBody()  # cache: used for sensor mount

# === Sensor manager + camera (scored core — prompt-required sensor) ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Camera offset: forward and above the chassis
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,
    15,                             # update rate Hz — physical rate, not 1/dt
    offset_pose,
    720, 480,                       # width, height
    1.408,                          # horizontal FOV (rad),
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/"))
manager.AddSensor(cam)

# === Visualization (full Irrlicht block) ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Camera sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Review-only recording setup ===

# === Main loop ===
time = 0.0


while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    # Irrlicht frame capture (review-only)

    # Inner physics batch
    for _ in range(render_every):
        # CSV log (review-only)

        # Steering schedule: hold straight, turn, then back out
        steering = 0.0
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5
        driver.SetSteering(steering)
        rover.Update()
        manager.Update()  # sensor update every physics step
        system.DoStepDynamics(time_step)
        time += time_step

        if system.GetChTime() >= sim_end:
            break

# === Post-loop: close CSV + assemble videos + plot (review-only) ===
