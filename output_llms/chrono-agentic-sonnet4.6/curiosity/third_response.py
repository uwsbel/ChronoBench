"""
Curiosity Mars Rover simulation with lidar sensor on rigid terrain.

Models the NASA Curiosity Mars rover (robot.Curiosity) navigating on a
rigid flat ground using a DC motor steering controller. A lidar sensor is
mounted on the rover chassis to scan the environment. The rover uses
ChSystemNSC with Bullet collision. Irrlicht provides interactive
visualization. The lidar sensor produces depth and point cloud data
visualized in a live preview window.

System: ChSystemNSC
Rover: robot.Curiosity with CuriosityDCMotorControl
Terrain: rigid ChBodyEasyBox ground
Sensor: ChLidarSensor chassis-mounted with point cloud visualization
Expected behavior: rover rolls forward with gentle steering; lidar scans
environment; live Irrlicht window + lidar point-cloud visualization open.
"""

import math
import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


# === Constants ===
TIME_STEP = 1e-3                # physics step: 1 ms (Curiosity/Viper standard)
SIM_END   = 20.0                # simulation duration (s)
RENDER_FPS = 50.0               # Irrlicht render rate
RENDER_EVERY = max(1, round(1.0 / (RENDER_FPS * TIME_STEP)))  # precomputed once

# Lidar parameters (all precomputed once)
LIDAR_UPDATE_RATE   = 5.0       # Hz — physical sensor update rate
LIDAR_H_SAMPLES     = 800       # horizontal samples
LIDAR_V_SAMPLES     = 300       # vertical samples
LIDAR_H_FOV         = 2 * chrono.CH_PI   # 360° horizontal
LIDAR_MAX_VERT_ANGLE =  chrono.CH_PI / 12   # +15° up
LIDAR_MIN_VERT_ANGLE = -chrono.CH_PI / 6    # -30° down
LIDAR_MAX_RANGE     = 100.0     # metres
LIDAR_COLLECTION_WINDOW = 1.0 / LIDAR_UPDATE_RATE  # 0.2 s per scan

MAX_STEERING = math.pi / 6      # max steering ramp (rad)

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
# Curiosity sits low: ground box top at z=0.0, box centre at z=-0.5
ground = chrono.ChBodyEasyBox(30, 30, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(
    chrono.GetChronoDataFile("textures/concrete.jpg")
)
system.Add(ground)

# === Rover (Curiosity) ===
rover  = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)

init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))

chassis_body = rover.GetChassis().GetBody()   # cache: fetched once, reused every step

# === Sensor manager + lidar ===
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)
manager.scene.AddPointLight(
    chrono.ChVector3f(-5, -2.5, 100),
    chrono.ChColor(1, 1, 1),
    500.0,
)

# Lidar mounted on rover chassis — looking forward from slightly above chassis centre
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
lidar = sens.ChLidarSensor(
    chassis_body,           # attach to rover chassis
    LIDAR_UPDATE_RATE,      # update rate (Hz)
    lidar_offset_pose,      # offset pose on chassis
    LIDAR_H_SAMPLES,        # horizontal samples
    LIDAR_V_SAMPLES,        # vertical samples
    LIDAR_H_FOV,            # horizontal FOV (rad)
    LIDAR_MAX_VERT_ANGLE,   # max vertical angle (rad)
    LIDAR_MIN_VERT_ANGLE,   # min vertical angle (rad)
    LIDAR_MAX_RANGE,        # max range (m)
    sens.LidarBeamShape_RECTANGULAR,
    2,                      # sample radius
    0.003,                  # vertical divergence angle (rad)
    0.003,                  # horizontal divergence angle (rad)
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Chassis Lidar")
lidar.SetLag(0)
lidar.SetCollectionWindow(LIDAR_COLLECTION_WINDOW)

# Lidar filter chain: visualize raw depth, access depth+intensity,
# convert to XYZ point cloud, visualize point cloud, access XYZI
lidar.PushFilter(sens.ChFilterVisualize(LIDAR_H_SAMPLES, LIDAR_V_SAMPLES, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# === Visualization (Irrlicht) ===
# Initialize() FIRST, then scene elements AFTER (Irrlicht call-order contract)
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover with Lidar Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.0, 2.0), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(
    chrono.ChVector3d(1.5, -2.5, 5.5),
    chrono.ChVector3d(0, 0, 0.5),
    3, 4, 10, 40, 512,
)

# === Review-only: recording setup ===

# === Main loop ===
frame = 0
try:
    while vis.Run() and system.GetChTime() < SIM_END:
        time = system.GetChTime()  # cache: read once per outer-loop iteration

        # Ramp steering: drive straight, turn, come back
        steering = 0.0
        if 2.0 < time < 7.0:
            steering = MAX_STEERING * (time - 2.0) / 5.0       # ramp up
        elif 7.0 < time < 12.0:
            steering = MAX_STEERING * (12.0 - time) / 5.0      # ramp down
        driver.SetSteering(steering)
        rover.Update()          # propagate steering into DC motors (required)

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(RENDER_EVERY):
            manager.Update()            # update all sensors once per physics step
            system.DoStepDynamics(TIME_STEP)
            if system.GetChTime() >= SIM_END:
                break

except (RuntimeError, ValueError) as exc:   # solver divergence / bad simulation state
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
