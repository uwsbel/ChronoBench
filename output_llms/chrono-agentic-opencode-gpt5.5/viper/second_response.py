"""Viper rover with a chassis-mounted camera sensor on rigid terrain.

The simulation uses a non-smooth contact system with Bullet collision, a fixed
textured ground plane, the built-in Viper rover with its DC motor driver, and a
front third-person camera sensor attached to the chassis. The rover drives on the
terrain while the sensor manager updates the camera stream and the Irrlicht view
renders at a fixed visual cadence.
"""

import math

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants ===
# Named values keep the rover, camera, and render cadence easy to audit.
time_step = 1.0e-3
sim_end = 8.0
render_step_size = 1.0 / 25.0
render_steps = math.ceil(render_step_size / time_step)  # precomputed once
ground_size_x = 20.0
ground_size_y = 20.0
ground_thickness = 1.0
ground_density = 1000.0
ground_z = -1.0
rover_start = chrono.ChVector3d(0.0, 0.2, 0.0)
rover_rotation = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
camera_width = 720
camera_height = 480
camera_update_rate = 15.0
camera_fov = 1.408
camera_offset = chrono.ChVector3d(1.0, 0.0, 1.45)
camera_tilt = 0.2
max_steering = math.pi / 8.0


# === System & Ground ===
# NSC contact and Bullet collision match the built-in rover contact model.
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.05)
ground = chrono.ChBodyEasyBox(
    ground_size_x,
    ground_size_y,
    ground_thickness,
    ground_density,
    True,
    True,
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, ground_z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === Rover ===
# The Viper object builds its own chassis, wheels, suspension, joints, and motors.
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(rover_start, rover_rotation))
chassis_body = rover.GetChassis().GetBody()  # cache: reused for sensors and logs


# === Sensor Manager & Camera ===
# The camera is a real chassis-mounted OptiX sensor, not the Irrlicht review camera.
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2.0, 2.5, 100.0),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

offset_pose = chrono.ChFramed(
    camera_offset,
    chrono.QuatFromAngleAxis(camera_tilt, chrono.ChVector3d(0.0, 1.0, 0.0)),
)
cam = sens.ChCameraSensor(
    chassis_body,
    camera_update_rate,
    offset_pose,
    camera_width,
    camera_height,
    camera_fov,
)
cam.SetName("Third Person POV")
cam.SetLag(0.0)
cam.SetCollectionWindow(0.0)
cam.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "Viper Front Camera"))
manager.AddSensor(cam)


# === Visualization ===
# Irrlicht is initialized before scene elements are added, per the renderer contract.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover with chassis camera")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-4.0, -6.0, 3.0), chrono.ChVector3d(2.5, 1.5, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0.0, 0.0, 0.5),
    3,
    4,
    10,
    40,
    512,
)
vis.AddGrid(1.0, 1.0, 20, 20, chrono.ChCoordsysd(), chrono.ChColor(0.35, 0.35, 0.35))

# === Main Loop ===
# The rover updates, sensors tick, and Irrlicht renders at the requested cadence.
step_number = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        time = system.GetChTime()
        steering = 0.0
        if 2.0 < time < 5.0:
            steering = max_steering * (time - 2.0) / 3.0
        elif 5.0 <= time < 8.0:
            steering = max_steering * (8.0 - time) / 3.0

        driver.SetSteering(steering)
        rover.Update()
        manager.Update()

        if step_number % render_steps == 0:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()


        step_number += 1
        system.DoStepDynamics(time_step)

except (RuntimeError, ValueError, OSError) as exc:  # solver divergence / output path errors
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass
