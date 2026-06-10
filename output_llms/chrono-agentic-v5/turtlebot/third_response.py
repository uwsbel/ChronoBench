"""TurtleBot differential-drive robot on rigid terrain with an onboard 2D lidar.

System type: ChSystemNSC (non-smooth) with Bullet collision — the canonical setup
for PyChrono's built-in rovers. The TurtleBot is a system-owned, fully-modeled
2-wheel differential-drive bot whose chassis carries a forward-facing lidar sensor.
The scene also contains five randomly placed box obstacles for the lidar to scan
and for the bot to interact with. A move() helper commands the two drive wheels;
here the bot drives straight forward across the ground, scanning the obstacles.

Expected behavior: the TurtleBot rolls forward in a straight line on the ground
(ground top at z=-0.1), the lidar continuously sweeps a 360 deg horizontal point
cloud of the surrounding boxes, and the boxes rest on the ground.
"""

import os
import math
import random

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / control parameters
time_step = 2e-3                 # TurtleBot integrates at 2e-3
sim_end = 12.0
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # precomputed once

GROUND_Z = -0.6                  # ground body center (box is 1 m thick -> top at -0.1)
WHEEL_SPEED = 2 * math.pi        # rad/s per drive wheel for visible forward motion

LEFT_DRIVE_WHEEL = 0             # WheelID: 0 = LEFT
RIGHT_DRIVE_WHEEL = 1            #          1 = RIGHT

N_BOXES = 5                      # number of random obstacle boxes
BOX_SIZE = 0.4                   # box edge length (m)


os.makedirs("cam", exist_ok=True)   # guard against missing output dir

# === System & gravity === NSC + Bullet collision (required: wheel<->ground contact)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed box, top surface under the bot at z = GROUND_Z + 0.5
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, GROUND_Z))
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Obstacle boxes === five randomly placed dynamic boxes resting on the ground
random.seed(7)                                  # deterministic placement
ground_top = GROUND_Z + 0.5
box_mat = chrono.ChContactMaterialNSC()
for i in range(N_BOXES):
    bx = random.uniform(-3.0, 3.0)
    by = random.uniform(-3.0, 3.0)
    box = chrono.ChBodyEasyBox(BOX_SIZE, BOX_SIZE, BOX_SIZE, 200, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, ground_top + BOX_SIZE / 2))
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.5 + 0.1 * i, 0.7))
    system.Add(box)

# === Robot === built-in TurtleBot (pose in constructor, no-arg Initialize)
init_pos = chrono.ChVector3d(0, 0.2, 0)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)     # identity (w, x, y, z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)
robot_tb.Initialize()
# cache: chassis body fetched once by name (TurtleBot owns its bodies), reused
chassis_body = next(b for b in system.GetBodies() if b.GetName() == "chassis_body")

# === Sensors === manager + forward-facing 2D lidar mounted on the chassis
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

H_SAMPLES = 800                  # horizontal lidar samples
V_SAMPLES = 1                    # 2D lidar -> single vertical layer
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.3),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    chassis_body,                # attach to chassis
    5.0,                         # update_rate (Hz)
    lidar_offset,                # offset pose
    H_SAMPLES,                   # horizontal samples
    V_SAMPLES,                   # vertical samples (1 -> 2D)
    2 * chrono.CH_PI,            # horizontal_fov
    0.0,                         # max_vert_angle (2D -> 0)
    0.0,                         # min_vert_angle (2D -> 0)
    100.0,                       # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                           # sample_radius
    0.003,                       # vert divergence
    0.003,                       # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)
lidar.PushFilter(sens.ChFilterVisualize(H_SAMPLES, 480, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)


# === Motion control === drive the two wheels per requested mode
def move(mode):
    """Command the TurtleBot wheels: 'straight', 'left', or 'right'."""
    if mode == "straight":
        robot_tb.SetMotorSpeed(-WHEEL_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-WHEEL_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "left":
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-WHEEL_SPEED, RIGHT_DRIVE_WHEEL)
    elif mode == "right":
        robot_tb.SetMotorSpeed(-WHEEL_SPEED, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:
        raise ValueError("unknown move mode: %r" % mode)


# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - rigid terrain with lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

# === Main loop === drive straight; update lidar + physics each step

frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        move("straight")

        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()
            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:   # solver divergence / bad command
    import traceback
    traceback.print_exc()
    raise
finally:
    pass

# === Post-processing ===
