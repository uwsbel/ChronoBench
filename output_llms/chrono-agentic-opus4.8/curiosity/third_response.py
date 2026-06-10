"""Curiosity Mars rover on rigid terrain with a chassis-mounted lidar sensor.

System: non-smooth (ChSystemNSC) with Bullet collision (wheel-terrain contact).
Main bodies: the built-in robot.Curiosity rocker-bogie rover (owns its chassis,
wheels, suspension and motors) plus a fixed rigid ground box. A ChLidarSensor is
mounted on the rover chassis through a ChSensorManager and produces a live depth +
point-cloud stream.

Expected behavior: the DC-motor-driven rover rolls forward across the rigid ground;
the chassis-mounted lidar continuously scans the surrounding scene, yielding a depth
image and an XYZI point cloud each sensor tick.
"""

import os
import math
import pychrono.core as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# === Named constants === geometry / physics / sensor parameters
time_step = 1e-3                     # rover integration step
sim_end = 12.0                       # total simulated seconds
render_fps = 50.0                    # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))   # precomputed once
max_steering = math.pi / 6           # practical rover steering limit (rad)

# Lidar parameters (chassis-mounted)
lidar_update_rate = 5.0              # Hz — physical lidar rate, not 1/dt
lidar_h_samples = 800                # horizontal beams
lidar_v_samples = 300                # vertical beams
lidar_h_fov = 2 * chrono.CH_PI       # full 360 deg horizontal field of view
lidar_max_vert = chrono.CH_PI / 12   # upper vertical angle
lidar_min_vert = -chrono.CH_PI / 6   # lower vertical angle
lidar_max_range = 100.0              # m


# === System & gravity === NSC + Bullet collision for wheel-terrain contact
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # Z-up world
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# === Ground === fixed rigid box; top surface at z=0 under the Curiosity spawn
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))     # Curiosity sits lower: box top at 0.0
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# === Rover === built-in Curiosity (owns its bodies); DC-motor steering driver
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)               # SetDriver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0, 0.2)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)      # identity (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))   # Initialize takes a ChFramed
chassis_body = rover.GetChassis().GetBody()      # cache: chassis body, reused for sensor + log

# === Sensor === chassis-mounted lidar via a ChSensorManager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(1, 1, 1), 500.0)

# Mount the lidar on the chassis, raised above the deck and looking forward.
lidar_offset = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.5),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    chassis_body,                     # attach to rover chassis
    lidar_update_rate,                # update rate (Hz)
    lidar_offset,                     # offset pose on the chassis
    lidar_h_samples,                  # horizontal samples
    lidar_v_samples,                  # vertical samples
    lidar_h_fov,                      # horizontal FOV (rad)
    lidar_max_vert,                   # max vertical angle
    lidar_min_vert,                   # min vertical angle
    lidar_max_range,                  # max range (m)
    sens.LidarBeamShape_RECTANGULAR,  # beam shape
    2,                                # sample radius
    0.003,                            # vertical divergence angle
    0.003,                            # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)   # lidar collection window = 1/rate

# Lidar filter chain (ORDER MATTERS): raw depth -> point cloud streams.
lidar.PushFilter(sens.ChFilterVisualize(lidar_h_samples, lidar_v_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())            # host access to depth + intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())          # host access to XYZI
manager.AddSensor(lidar)

# === Visualization === full Irrlicht scene: window + sky + camera + lights
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - chassis lidar")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)   # pos, aim, radius, near, far, angle, resolution

# === Main loop === drive forward; pump the lidar each step; capture review video
os.makedirs("cam", exist_ok=True)            # guard against missing output dir

frame = 0
try:
    while vis.Run() and system.GetChTime() < sim_end:
        # Drive straight ahead (the DC-motor drive is always on; keep steering neutral).
        driver.SetSteering(0.0)
        rover.Update()           # REQUIRED: propagate driver command into the motors

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        for _ in range(render_every):
            manager.Update()     # pump the lidar exactly once per physics step
            system.DoStepDynamics(time_step)
            if system.GetChTime() >= sim_end:
                break
except (RuntimeError, ValueError) as exc:    # solver divergence / invalid state
    import traceback
    traceback.print_exc()
    raise
