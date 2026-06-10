"""
Curiosity Mars rover simulation with lidar sensor on the chassis.

System type: ChSystemNSC with Bullet collision.
Bodies: fixed rigid ground (NSC contact material) + Curiosity rover (built-in robot.Curiosity).
Sensor: ChLidarSensor mounted on the rover chassis — horizontal/vertical scan with
        ChFilterDIAccess, ChFilterPCfromDepth, ChFilterVisualizePointCloud, ChFilterXYZIAccess.
Sensor manager: ChSensorManager with point lights for rendering.
Expected behavior: Curiosity rover drives forward with straight steering; the lidar sensor
                   updates at 5 Hz, scanning the environment and visualizing the point cloud.
"""

# === Imports ===
import math
import os
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# === Constants ===
TIME_STEP = 1e-3          # physics time step (s)
SIM_END = 20.0            # simulation end time (s)
RENDER_FPS = 50.0         # Irrlicht render rate (Hz)
render_every = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Lidar parameters
LIDAR_UPDATE_RATE = 5.0       # Hz — physical scan rate
LIDAR_H_SAMPLES = 800         # horizontal samples
LIDAR_V_SAMPLES = 300         # vertical samples
LIDAR_H_FOV = 2 * chrono.CH_PI  # full 360° horizontal FOV
LIDAR_MAX_VERT = chrono.CH_PI / 12    # max vertical angle (rad)
LIDAR_MIN_VERT = -chrono.CH_PI / 6   # min vertical angle (rad)
LIDAR_MAX_RANGE = 100.0       # max range (m)

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
ground = chrono.ChBodyEasyBox(40, 40, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # Curiosity: ground z=-0.5, top at 0.0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Curiosity rover ===
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)  # identity (w, x, y, z)
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
manager.scene.AddPointLight(
    chrono.ChVector3f(9, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# === Lidar sensor (chassis-mounted) ===
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-12, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,              # attach to rover chassis
    LIDAR_UPDATE_RATE,         # update rate (Hz)
    lidar_offset_pose,         # offset pose on chassis
    LIDAR_H_SAMPLES,           # horizontal samples
    LIDAR_V_SAMPLES,           # vertical samples
    LIDAR_H_FOV,               # horizontal FOV (rad)
    LIDAR_MAX_VERT,            # max vertical angle (rad)
    LIDAR_MIN_VERT,            # min vertical angle (rad)
    LIDAR_MAX_RANGE,           # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                         # sample radius
    0.003,                     # vertical divergence angle
    0.003,                     # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / LIDAR_UPDATE_RATE)  # collection window = 1/update_rate

# Lidar filter chain (ORDER MATTERS)
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())           # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar)

# === Visualization ===
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover with Lidar Sensor")
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

# === Review-only setup ===

# === Main loop ===
frame = 0  # consecutive frame counter for review video


try:
    while vis.Run() and system.GetChTime() < SIM_END:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            sim_time = system.GetChTime()  # cache: read once per inner step

            # Rover steering: drive straight
            driver.SetSteering(0.0)
            rover.Update()


            manager.Update()          # update all sensors each physics step
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:  # solver divergence / bad state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
