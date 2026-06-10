"""
Viper Rover simulation with a chassis-mounted camera sensor.

System type: ChSystemNSC (NSC contact, Bullet collision).
Main bodies: rigid ground plane, Viper rover (6-wheel suspension, DC motor drive).
Sensor: ChCameraSensor attached to the rover chassis — Third Person POV camera
        at 15 Hz, 720x480, FOV=1.408 rad, with ChFilterVisualize live preview.
Expected behavior: Viper rolls forward on flat rigid terrain; the sensor manager
pumps the OptiX camera each physics step; Irrlicht window renders the scene.
"""

import math
import os
import csv

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants ===
TIME_STEP     = 1e-3          # physics time step (s)
SIM_END       = 20.0          # simulation duration (s)
RENDER_FPS    = 25.0          # Irrlicht render rate (fps)
# precomputed once
RENDER_EVERY  = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))

# Sensor / render-step sizing (from prompt)
RENDER_STEP_SIZE = 1.0 / 25   # 25 FPS for step-number gating  # precomputed once
RENDER_STEPS     = math.ceil(RENDER_STEP_SIZE / TIME_STEP)       # precomputed once

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

ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1))   # top surface at z = 0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Viper rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0, 0.5)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused each step

# === Sensor manager & camera ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Camera mounted on the rover chassis — Third Person POV
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,   # attach to chassis body
    15,             # update rate (Hz) — physical rate
    offset_pose,
    720,            # image width
    480,            # image height
    1.408,          # horizontal FOV (rad)
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === Visualization (Irrlicht) ===
# Initialize() FIRST, then scene elements (Irrlicht call order)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover — Camera Sensor Demo")
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)
vis.AddGrid(
    0.5, 0.5, 40, 40,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
    chrono.ChColor(0.4, 0.4, 0.4),
)

# === Review-only setup ===


# === Main loop ===
step_number = 0   # tracks physics steps for render-cadence gating

try:
    frame = 0
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: current sim time for this frame

        # Render the Irrlicht scene at the gated render cadence
        if step_number % RENDER_STEPS == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        # Advance one physics step: update rover steering, sensor manager, dynamics
        driver.SetSteering(0.0)
        rover.Update()
        manager.Update()


        system.DoStepDynamics(TIME_STEP)
        step_number += 1

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise

finally:
    pass
