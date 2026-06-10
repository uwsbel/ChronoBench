"""Curiosity rover on rigid terrain using PyChrono NSC contact.

The script builds a Bullet-backed NSC system with a fixed textured ground body,
the built-in Curiosity rover, and a Curiosity DC motor steering driver. The rover
rolls forward under its motor model while the steering command ramps smoothly to
show real-time turning on rigid terrain in an Irrlicht window with sky, lighting,
shadowing, logo, grid, and a fixed chase-style camera.
"""

import math

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot


# === Constants === named values keep the terrain, camera, and control setup explicit
TIME_STEP = 1.0e-3
SIM_END = 4.0
RENDER_FPS = 30.0
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once
GROUND_SIZE_X = 20.0
GROUND_SIZE_Y = 20.0
GROUND_THICKNESS = 1.0
GROUND_DENSITY = 1000.0
GROUND_Z = -0.5
GROUND_FRICTION = 0.8
GROUND_RESTITUTION = 0.0
INIT_POS = chrono.ChVector3d(0.0, 0.2, 0.0)
INIT_ROT = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)
MAX_STEERING = math.pi / 6.0
DRIVE_NO_LOAD_SPEED = 6.0
DRIVE_STALL_TORQUE = 120.0
STEER_START = 0.75
STEER_RAMP_END = 2.25
STEER_RELEASE_END = 3.75
CAMERA_EYE = chrono.ChVector3d(-3.5, -5.0, 2.2)
CAMERA_TARGET = chrono.ChVector3d(1.0, 0.0, 0.6)
SHADOW_LIGHT_POS = chrono.ChVector3d(1.5, -2.5, 5.5)
SHADOW_LIGHT_AIM = chrono.ChVector3d(0.0, 0.0, 0.5)


def steering_command(time):
    """Return a smooth Curiosity steering command in radians."""
    if STEER_START < time < STEER_RAMP_END:
        return MAX_STEERING * (time - STEER_START) / (STEER_RAMP_END - STEER_START)
    if STEER_RAMP_END <= time < STEER_RELEASE_END:
        return MAX_STEERING * (STEER_RELEASE_END - time) / (STEER_RELEASE_END - STEER_RAMP_END)
    return 0.0


# === System & Collision === NSC rover contact needs Bullet collision and small margins
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


# === Terrain === fixed rigid support gives the rover a collidable textured surface
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(GROUND_FRICTION)
ground_mat.SetRestitution(GROUND_RESTITUTION)
ground = chrono.ChBodyEasyBox(
    GROUND_SIZE_X,
    GROUND_SIZE_Y,
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


# === Rover & Driver === built-in Curiosity creates chassis, wheels, bogies, joints, and motors
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
for wheel_id in (robot.C_LF, robot.C_LM, robot.C_LB, robot.C_RF, robot.C_RM, robot.C_RB):
    driver.SetMotorNoLoadSpeed(DRIVE_NO_LOAD_SPEED, wheel_id)
    driver.SetMotorStallTorque(DRIVE_STALL_TORQUE, wheel_id)
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(INIT_POS, INIT_ROT))
chassis_body = rover.GetChassis().GetBody()  # cache: chassis pose is logged every step


# === Visualization === Irrlicht is initialized before adding sky, camera, logo, and lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - rigid terrain")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(CAMERA_EYE, CAMERA_TARGET)
vis.AddTypicalLights()
vis.AddLightWithShadow(SHADOW_LIGHT_POS, SHADOW_LIGHT_AIM, 3, 4, 10, 40, 512)
vis.AddGrid(
    2.0,
    2.0,
    20,
    20,
    chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.01), chrono.QUNIT),
    chrono.ChColor(0.35, 0.35, 0.35),
)


# === Main Loop === steer, update Curiosity motor controls, render, and advance dynamics
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            sim_time = system.GetChTime()
            steering = steering_command(sim_time)
            driver.SetSteering(steering)
            rover.Update()
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:  # solver divergence or invalid rover state
    raise
except (OSError, IOError) as exc:  # output directory or recording file failure
    raise
finally:
    pass
