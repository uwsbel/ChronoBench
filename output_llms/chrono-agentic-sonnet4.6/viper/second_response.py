"""
Viper rover simulation on rigid terrain with a chassis-mounted camera sensor.

System type: ChSystemNSC (non-smooth contact, Z-up).
Main bodies: flat rigid ground (ChBodyEasyBox), Viper rover (robot.Viper with
ViperDCMotorControl driver).

Expected behavior: The Viper rover drives forward on rigid terrain while a
ChCameraSensor mounted on the chassis provides a third-person POV preview.
The sensor manager is updated every physics step; rendering is throttled to
25 Hz for efficiency.
"""

# === Imports ===
import math
import os
import csv
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Simulation parameters ===
TIME_STEP = 1e-3          # physics step size (s)
SIM_END   = 20.0          # simulation duration (s)
RENDER_FPS = 25.0         # Irrlicht render rate (Hz)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground ===
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # top surface at z=0
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

chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for sensor mount

# === Sensor manager ===
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Camera sensor (chassis-mounted, third-person POV) ===
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,   # attach to rover chassis
    15,             # update rate (Hz) — physical rate
    offset_pose,
    720, 480,       # width, height
    1.408,          # horizontal FOV (rad)
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - Rigid terrain with camera sensor")
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
max_steering = math.pi / 6
frame = 0
step_number = 0
render_steps = math.ceil((1.0 / RENDER_FPS) / TIME_STEP)  # precomputed once


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()

        time = system.GetChTime()

        # Ramp steering: hold straight, ramp up, then ramp down
        steering = 0.0
        if 2.0 < time < 7.0:
            steering = max_steering * (time - 2.0) / 5.0
        elif 7.0 < time < 12.0:
            steering = max_steering * (12.0 - time) / 5.0
        driver.SetSteering(steering)

        rover.Update()        # propagate steering to motors
        manager.Update()      # pump sensor (every physics step)


        system.DoStepDynamics(TIME_STEP)
        step_number += 1

except (RuntimeError, ValueError) as exc:   # solver divergence / bad state
    import traceback; traceback.print_exc()
    raise
