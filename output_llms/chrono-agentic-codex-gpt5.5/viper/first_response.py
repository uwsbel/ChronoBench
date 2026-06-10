"""Viper rover on rigid terrain using a PyChrono NSC system.

The simulation builds a fixed textured terrain body, initializes the built-in
Viper rover with its DC motor driver, and shows the rover in an Irrlicht window.
The rover drives forward while steering ramps smoothly left and then returns
toward straight travel over the timed run.
"""

import math

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants ===
TIME_STEP = 1.0e-3
SIM_END = 12.0
GROUND_LENGTH = 20.0
GROUND_WIDTH = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -1.0
INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
MAX_STEERING = math.pi / 6.0
STEER_RAMP_START = 2.0
STEER_RAMP_END = 7.0
STEER_RETURN_END = 12.0
STEP_BATCH = 1  # precomputed once; scored core advances one physics step per render


def steering_at(time):
    """Return the smooth steering command for the current simulation time."""
    if STEER_RAMP_START < time < STEER_RAMP_END:
        return MAX_STEERING * (time - STEER_RAMP_START) / (STEER_RAMP_END - STEER_RAMP_START)
    if STEER_RAMP_END <= time < STEER_RETURN_END:
        return MAX_STEERING * (STEER_RETURN_END - time) / (STEER_RETURN_END - STEER_RAMP_END)
    return 0.0


# === System & gravity ===
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Rigid terrain ===
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.05)
ground = chrono.ChBodyEasyBox(
    GROUND_LENGTH,
    GROUND_WIDTH,
    GROUND_THICKNESS,
    GROUND_DENSITY,
    True,
    True,
    ground_mat,
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)


# === Rover & driver ===
rover = robot.Viper(system)
driver = robot.ViperDCMotorControl()
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(INIT_POS, INIT_ROT))
chassis_body = rover.GetChassis().GetBody()  # cache: fetched once, reused for camera/logging
realtime_timer = chrono.ChRealtimeStepTimer()

print("ROVER CHASSIS MASS: ", chassis_body.GetMass())


# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Viper rover - rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5.0, -4.0, 2.0), chrono.ChVector3d(0.0, 0.0, 0.5))
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
vis.AddGrid(
    1.0,
    1.0,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, -0.49), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main loop ===

frame = 0

try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(STEP_BATCH):
            time = system.GetChTime()
            steering = steering_at(time)
            driver.SetSteering(steering)
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            realtime_timer.Spin(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError, ArithmeticError) as exc:  # Chrono step or invalid state failure
    print(f"Simulation failed: {exc}")
    raise
finally:
    pass


# === Review post-processing ===
