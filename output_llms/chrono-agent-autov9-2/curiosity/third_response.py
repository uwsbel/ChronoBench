"""Curiosity Mars rover driving on flat rigid ground, with a chassis-mounted lidar.

Models the NASA Curiosity rover (pychrono.robot.Curiosity) driven by a DC-motor
controller on a flat rigid-terrain plate. The system is an NSC multibody system
with Bullet collision between the rover wheels and the ground, so the rover rolls
forward under wheel-motor torque. A rotating 2D lidar (ChLidarSensor) is rigidly
attached to the rover chassis to range the surrounding scene; its depth/intensity
(DI), point-cloud (PCfromDepth) and XYZI buffers are exposed through access
filters and pumped by a ChSensorManager.

Expected behavior: the six-wheeled rover accelerates from rest and translates
forward across the ground over the simulated interval; the lidar produces a
populated depth/point-cloud buffer each tick.
"""

import os
import math
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.robot as robot
import pychrono.sensor as sens

# === Constants === geometry / physics / timing (no bare literals downstream)
TIME_STEP = 2.0e-3            # solver step (s)
SIM_END = 6.0                # simulated duration (s)
RENDER_FPS = 20.0            # review render cadence (frames/s)
GROUND_SIZE = 30.0           # square ground plate side length (m)
GROUND_THICK = 1.0           # ground plate thickness (m)
GROUND_Z = -0.5 * GROUND_THICK   # plate top surface sits at z = 0
ROVER_START = chrono.ChVector3d(0.0, 0.0, 0.0)   # chassis spawn (above ground top)
MOTOR_NO_LOAD_SPEED = math.pi    # wheel-motor no-load speed (rad/s)
MOTOR_STALL_TORQUE = 300.0       # wheel-motor stall torque (N*m)
LIDAR_UPDATE_RATE = 5.0          # lidar revolutions reported per second (Hz)
LIDAR_HORIZ_SAMPLES = 180        # horizontal samples per scan
LIDAR_VERT_SAMPLES = 8           # vertical channels
LIDAR_HFOV = 2.0 * math.pi       # full 360 deg horizontal field of view (rad)
LIDAR_MAX_VERT = 0.15            # max vertical beam angle (rad)
LIDAR_MIN_VERT = -0.15           # min vertical beam angle (rad)
LIDAR_MAX_DIST = 40.0            # max range (m)
LIDAR_OFFSET = chrono.ChVector3d(0.0, 0.0, 0.4)  # mast offset on chassis (m)

RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))   # precomputed once
LIDAR_UPDATE_RATE_HZ = 1.0 / TIME_STEP                          # precomputed once


# === System & gravity === NSC system with Bullet collision for wheel/ground contact
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # rover<->ground contact

# === Ground === fixed rigid plate the rover drives on (gives the wheels contact)
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.9)
ground_mat.SetRestitution(0.0)
ground = chrono.ChBodyEasyBox(GROUND_SIZE, GROUND_SIZE, GROUND_THICK,
                              1000.0, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
sys.Add(ground)

# === Rover === Curiosity with a DC-motor driver; per-wheel motor speed/torque set
rover = robot.Curiosity(sys)
driver = robot.CuriosityDCMotorControl()
for wheel_id in (robot.C_LF, robot.C_RF, robot.C_LM, robot.C_RM, robot.C_LB, robot.C_RB):
    driver.SetMotorNoLoadSpeed(MOTOR_NO_LOAD_SPEED, wheel_id)   # per-wheel motor id
    driver.SetMotorStallTorque(MOTOR_STALL_TORQUE, wheel_id)
driver.SetSteering(0.0)        # drive straight forward
rover.SetDriver(driver)
rover.Initialize(chrono.ChFramed(ROVER_START, chrono.QUNIT))
rover.Update()   # settle the wheel motors before stepping

chassis = rover.GetChassis().GetBody()   # cache: chassis body fetched once, reused every step

# === Sensors === chassis-mounted rotating lidar with DI / point-cloud / XYZI access
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(0, 0, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

lidar = sens.ChLidarSensor(
    chassis,                                  # rover chassis body the lidar rides on
    LIDAR_UPDATE_RATE,
    chrono.ChFramed(LIDAR_OFFSET, chrono.QUNIT),
    LIDAR_HORIZ_SAMPLES,
    LIDAR_VERT_SAMPLES,
    LIDAR_HFOV,
    LIDAR_MAX_VERT,
    LIDAR_MIN_VERT,
    LIDAR_MAX_DIST,
    sens.LidarBeamShape_RECTANGULAR,
    1,
    0.003,
    0.003,
    sens.LidarReturnMode_STRONGEST_RETURN,
    1.0e-3,
)
lidar.SetName("chassis_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())            # depth+intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())         # convert depth -> 3D point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())          # XYZI point-cloud buffer access
manager.AddSensor(lidar)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover with chassis lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-5, -5, 3), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(1.0, 1.0, 30, 30,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0.001), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))

# === Main loop === render-cadence outer loop; physics + sensors in the inner batch


frame = 0
try:
    while vis.Run() and sys.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(RENDER_EVERY):
            manager.Update()   # pump the lidar every physics step
            rover.Update()
            sys.DoStepDynamics(TIME_STEP)
            if sys.GetChTime() >= SIM_END:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad sensor state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing === assemble the Irrlicht review video, plot table, clean frames
