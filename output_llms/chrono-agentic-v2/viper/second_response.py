"""
Viper Rover Simulation with Chassis-Mounted Camera Sensor.

Models a Viper 6-wheel rover (robot.Viper) driving on rigid flat terrain
using a DC-motor steering driver (robot.ViperDCMotorControl). A sensor
manager with a chassis-mounted camera (ChCameraSensor) provides a
third-person POV with live preview. System type: ChSystemNSC (Z-up, Bullet).
Expected behavior: the rover rolls forward; the camera sensor updates at 15 Hz
and shows the "Viper Front Camera" preview window.
"""

import math
import os
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens

# === Constants ===
TIME_STEP   = 1e-3          # physics step (s)
SIM_END     = 20.0          # simulation end time (s)
RENDER_FPS  = 50.0          # Irrlicht render cadence (Hz)
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

MAX_STEERING = math.pi / 6  # maximum steering angle (rad)

# Camera sensor parameters
CAM_UPDATE_RATE = 15        # Hz — physical rate as in the reference
CAM_WIDTH       = 720
CAM_HEIGHT      = 480
CAM_FOV         = 1.408     # horizontal FOV (rad)

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

# === Rover ===
rover  = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)                     # MUST be before Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()   # cache: fetched once, reused for sensor

# === Sensor manager + chassis-mounted camera ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,          # attach to rover chassis
    CAM_UPDATE_RATE,       # physical update rate (Hz)
    offset_pose,
    CAM_WIDTH,
    CAM_HEIGHT,
    CAM_FOV,
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAM_WIDTH, CAM_HEIGHT, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))  # sensor output frames — scored core
manager.AddSensor(cam)

# === Visualization (Irrlicht) — Initialize FIRST, scene elements AFTER ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper Rover with Camera Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)


# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: current sim time for this frame

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            t = system.GetChTime()  # cache: updated each physics step

            # Ramp steering: straight, then turn in, then back out
            steering = 0.0
            if 2.0 < t < 7.0:
                steering = MAX_STEERING * (t - 2.0) / 5.0
            elif 7.0 < t < 12.0:
                steering = MAX_STEERING * (12.0 - t) / 5.0
            driver.SetSteering(steering)

            rover.Update()           # propagate steering to DC motors (REQUIRED)
            manager.Update()         # pump sensors every physics step


            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
