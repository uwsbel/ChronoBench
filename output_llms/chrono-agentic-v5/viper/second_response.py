"""Viper rover on rigid terrain with a chassis-mounted third-person camera sensor.

System type   : ChSystemNSC (non-smooth), Bullet collision — the Viper rover is
                tuned for NSC contact between its rigid wheels and the ground.
Main bodies   : a fixed rigid ground box and the built-in robot.Viper rover
                (chassis + six suspended wheels + DC-motor drive), driven by a
                ViperDCMotorControl steering driver.
Sensor        : an OptiX ChCameraSensor ("Third Person POV") rigidly mounted on
                the rover chassis via an offset pose, looking forward and slightly
                down, with a live visualize filter and an RGBA save stream.
Expected      : the rover rolls forward under its always-on DC drive while the
                onboard camera follows it from a third-person viewpoint.
"""

import os
import math

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Constants === geometry / physics / camera parameters (no bare literals downstream)
time_step = 1e-3
sim_end = 12.0
max_steering = math.pi / 6          # practical Viper steering limit (rad)
ground_z = -1.0                     # 1 m thick ground box -> top surface at z = -0.5
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity quaternion (w, x, y, z)

render_fps = 25.0                   # render cadence (Hz)
render_every = max(1, round(1.0 / (render_fps * time_step)))  # precomputed once

# === System & gravity === NSC + Bullet collision for rigid wheel/terrain contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up world
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed rigid box, top surface under the rover spawn at z = -0.5
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, ground_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Rover === built-in Viper; owns its bodies/joints/motors, DC steering driver
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)                      # SetDriver BEFORE Initialize
rover.Initialize(chrono.ChFramed(init_pos, init_rot))
chassis_body = rover.GetChassis().GetBody()  # cache: chassis fetched once, reused as sensor mount

# === Sensor === chassis-mounted third-person camera (OptiX) with point lighting
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)

offset_pose = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1.45),
                              chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))
cam = sens.ChCameraSensor(
    chassis_body,   # ride on the rover chassis
    15,             # update rate (Hz) — physical rate, not 1/dt
    offset_pose,
    720, 480,       # image width, height
    1.408,          # horizontal field of view (rad)
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(720, 480, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/third_person/"))
manager.AddSensor(cam)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - third person camera")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Main loop === ramp steering, pump sensors, advance the rover in real time
os.makedirs("cam", exist_ok=True)   # guard against missing output dir for sensor frames

frame = 0
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        t = system.GetChTime()

        # ramp steering: straight, turn in, then back out
        steering = 0.0
        if 2 < t < 7:
            steering = max_steering * (t - 2) / 5
        elif 7 < t < 12:
            steering = max_steering * (12 - t) / 5
        driver.SetSteering(steering)
        rover.Update()        # REQUIRED: propagate steering into the rover motors
        manager.Update()      # pump the camera sensor every step

        if step_number % render_every == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        system.DoStepDynamics(time_step)
        step_number += 1
except (RuntimeError, ValueError) as exc:   # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise

# === Post-processing === assemble review/sensor videos + plot the logged CSV table
