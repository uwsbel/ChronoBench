"""TurtleBot differential-drive robot driving straight across a ground plane,
sensed by an onboard 360-degree lidar, with five randomly placed boxes for
interaction.

System type : ChSystemNSC (rigid-contact, Bullet collision) — the robot wheels,
              the fixed ground, and the scattered boxes are all contact bodies.
Main bodies : a robot.TurtleBot differential-drive platform (two active wheels
              + caster), a large fixed ground box whose top sits at z = -0.6,
              and five free-falling/settling boxes scattered on the ground.
Sensor      : an onboard ChLidarSensor rigidly carried on the TurtleBot chassis,
              producing depth/intensity (DI), a point cloud (PCfromDepth), and
              XYZI access — pumped by a ChSensorManager every physics step.
Behavior    : both drive motors are commanded to the same forward speed via a
              move('straight') helper, so the robot rolls forward in +X while the
              lidar scans the surrounding boxes; left/right turning modes are
              available by driving the two wheels at different speeds.
"""

import os
import math

import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr


# === Named constants === geometry / physics / timing (no bare literals downstream)
time_step = 1.0e-3                  # 1 ms NSC step
sim_end = 10.0                      # seconds of simulated driving
render_fps = 30.0                   # review-render cadence
drive_speed = math.pi              # wheel angular speed (rad/s) for 'straight'
turn_speed = math.pi               # wheel angular speed used when turning

ground_top_z = -0.6                 # requested ground-plane top height
ground_thickness = 0.2              # ground box thickness
ground_size_xy = 20.0               # ground box footprint (square)
ground_center_z = ground_top_z - 0.5 * ground_thickness   # so the TOP face = ground_top_z

robot_z = ground_top_z + 0.0        # TurtleBot origin rides just above the ground top
robot_pos = chrono.ChVector3d(0.0, 0.2, robot_z)

box_size = 0.25                     # full edge length of each interaction box
box_mass = 1.0                      # interaction boxes are dynamic (free rigid bodies)
n_boxes = 5                         # five randomly placed boxes

render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once

# === System & gravity === NSC system; gravity along -Z (Z-up world)
sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0.0, 0.0, -9.81))
# Robot wheels + ground + boxes all collide -> Bullet narrow-phase is required.
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# === Contact material === shared NSC material for ground, robot wheels, and boxes
ground_mat = chrono.ChContactMaterialNSC()
ground_mat.SetFriction(0.8)
ground_mat.SetRestitution(0.0)

# === Bodies === fixed ground plane with its top face at z = ground_top_z
ground = chrono.ChBodyEasyBox(
    ground_size_xy, ground_size_xy, ground_thickness,
    1000.0, True, True, ground_mat,
)
ground.SetPos(chrono.ChVector3d(0.0, 0.0, ground_center_z))
ground.SetFixed(True)
ground.SetName("ground")
sys.Add(ground)

# === Interaction boxes === five randomly placed dynamic boxes on the ground
# Deterministic RNG so the scattered layout is reproducible run-to-run.
import random
rng = random.Random(7)
box_bodies = []                                          # cache: created once, reused for logging
box_drop_z = ground_top_z + 0.5 * box_size + 0.02        # rest just above the ground top
for i in range(n_boxes):
    # Scatter ahead of and beside the robot so the lidar sweeps them while driving.
    bx = rng.uniform(1.5, 6.0)
    by = rng.uniform(-2.0, 2.0)
    box = chrono.ChBodyEasyBox(box_size, box_size, box_size,
                               box_mass / (box_size ** 3), True, True, ground_mat)
    box.SetPos(chrono.ChVector3d(bx, by, box_drop_z))
    box.SetName(f"box_{i}")
    sys.Add(box)
    box_bodies.append(box)

# === Robot === differential-drive TurtleBot built on the shared NSC system
turtlebot = robot.TurtleBot(sys, robot_pos, chrono.QUNIT)
turtlebot.Initialize()

# Base/chassis body recovered from the system by name (created by the wrapper's
# Initialize()); fetch once and reuse every step rather than searching per step.
chassis = next(b for b in sys.GetBodies() if b.GetName() == "chassis_body")  # cache: TurtleBot chassis

# === Motion control === differential-drive helper mapping a mode to wheel speeds
# WheelID 0 = left active wheel (robot.LD), 1 = right active wheel (robot.RD).
def move(mode):
    """Command the two active drive wheels for a driving mode.

    'straight' drives both wheels forward at the same speed; 'left'/'right'
    spin the wheels in opposite directions to rotate the chassis in place.
    """
    if mode == "straight":
        turtlebot.SetMotorSpeed(-drive_speed, robot.LD)
        turtlebot.SetMotorSpeed(-drive_speed, robot.RD)
    elif mode == "left":
        turtlebot.SetMotorSpeed(turn_speed, robot.LD)
        turtlebot.SetMotorSpeed(-turn_speed, robot.RD)
    elif mode == "right":
        turtlebot.SetMotorSpeed(-turn_speed, robot.LD)
        turtlebot.SetMotorSpeed(turn_speed, robot.RD)
    else:
        raise ValueError(f"unknown drive mode: {mode!r}")   # guard: typo'd mode

# === Sensor manager === oversees the onboard lidar; lighting for the OptiX scene
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.SetAmbientLight(chrono.ChVector3f(0.3, 0.3, 0.3))

# === Lidar sensor === 360-degree onboard scanner rigidly carried on the chassis
lidar_update_rate = 10.0            # Hz
lidar_w = 360                       # horizontal samples
lidar_h = 16                        # vertical channels
lidar_hfov = 2.0 * math.pi          # full 360-degree horizontal sweep
lidar_max_vert = 0.2618             # +15 deg in radians
lidar_min_vert = -0.2618            # -15 deg in radians
lidar_max_dist = 40.0               # m
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, 0.3), chrono.QUNIT)  # 0.3 m above chassis

lidar = sens.ChLidarSensor(
    chassis,                        # parent body the lidar rides on
    lidar_update_rate,
    lidar_offset,
    lidar_w, lidar_h,
    lidar_hfov,
    lidar_max_vert, lidar_min_vert,
    lidar_max_dist,
    sens.LidarBeamShape_RECTANGULAR,
    1,                              # sample radius
    0.003, 0.003,                   # vertical / horizontal divergence
    sens.LidarReturnMode_MEAN_RETURN,
    1.0e-3,                         # clip near
)
lidar.SetName("onboard_lidar")
lidar.PushFilter(sens.ChFilterDIAccess())                # depth + intensity buffer access
lidar.PushFilter(sens.ChFilterPCfromDepth())             # depth image -> point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())              # XYZI point access
manager.AddSensor(lidar)

# === Visualization === full Irrlicht scene: window + sky + camera + lights + grid
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot driving straight with onboard lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-2.0, -3.0, 2.0), chrono.ChVector3d(2.0, 0.0, 0.0))
vis.AddTypicalLights()
vis.AddGrid(0.5, 0.5, 40, 40,
            chrono.ChCoordsysd(chrono.ChVector3d(0, 0, ground_top_z), chrono.QUNIT),
            chrono.ChColor(0.4, 0.4, 0.4))


# === Main loop === drive straight, pump the lidar each step, render at cadence
try:
    move("straight")                            # command a constant forward drive
    while vis.Run() and sys.GetChTime() < sim_end:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        for _ in range(render_every):
            manager.Update()                    # pump the lidar every physics step
            sys.DoStepDynamics(time_step)
            if sys.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:        # solver divergence / bad drive mode
    import traceback
    traceback.print_exc()
    raise
finally:
    pass
