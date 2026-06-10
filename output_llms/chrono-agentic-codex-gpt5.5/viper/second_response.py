"""Viper rover on rigid terrain with a chassis-mounted RGB camera sensor.

This PyChrono 9.0 NSC simulation builds the built-in Viper rover, a textured
rigid ground contact body, an Irrlicht scene, and a Chrono sensor camera
attached to the rover chassis. The rover drives forward under its DC motor
controller while the camera previews the third-person point of view.
"""

import math
import traceback

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens


# === Constants === named values keep the rover, camera, and recording coherent
TIME_STEP = 1.0e-3
SIM_END = 12.0
RENDER_FPS = 25.0
RENDER_EVERY = max(1, math.ceil((1.0 / RENDER_FPS) / TIME_STEP))  # precomputed once
CAMERA_RATE = 15
CAMERA_WIDTH = 720
CAMERA_HEIGHT = 480
CAMERA_FOV = 1.408
MAX_STEERING = math.pi / 6.0


# === System & terrain === NSC contact with Bullet supports the rover wheels
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(20.0, 20.0, 1.0, 1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -1.0))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === Rover === built-in Viper owns its chassis, suspension, wheels, and motors
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(chrono.ChVector3d(0, 0.2, 0), chrono.QUNIT))
chassis_body = rover.GetChassis().GetBody()  # cache: sensor mount and logs reuse the chassis


# === Sensor camera === OptiX camera rides on the rover chassis and updates each step
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
    chassis_body,
    CAMERA_RATE,
    offset_pose,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
)
cam.SetName("Third Person POV")
cam.SetLag(0)
cam.SetCollectionWindow(0)
cam.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Viper Front Camera"))
cam.PushFilter(sens.ChFilterRGBA8Access())
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))
manager.AddSensor(cam)


# === Visualization === Irrlicht scene is built after Initialize per PyChrono 9.0
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover camera sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-3.5, 3.0, 2.2), chrono.ChVector3d(0.8, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3,
    4,
    10,
    40,
    512,
)


# === Main loop === steering, rendering, sensor update, and dynamics advance together
def steering_command(time):
    """Return a smooth Viper steering command for stable visible motion."""
    if 2.0 < time < 7.0:
        return MAX_STEERING * (time - 2.0) / 5.0
    if 7.0 <= time < 12.0:
        return MAX_STEERING * (12.0 - time) / 5.0
    return 0.0


try:

    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            time = system.GetChTime()
            steering = steering_command(time)
            driver.SetSteering(steering)
            rover.Update()
            manager.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:
    traceback.print_exc()
    raise
except (OSError, IOError) as exc:
    traceback.print_exc()
    raise
finally:
    pass
