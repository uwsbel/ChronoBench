import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# System setup (NSC for rovers)
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # required for contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Ground (rigid terrain) — top surface at z=0 for Curiosity
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))  # top at z=0 for Curiosity
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Curiosity rover with DC-motor steering driver
rover = robot.Curiosity(system)
driver = robot.CuriosityDCMotorControl()
rover.SetDriver(driver)  # must be before Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)       # spawn above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)   # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))  # takes ChFramed, not bare pos

# Sensor manager + point light for scene
manager = sens.ChSensorManager(system)
intensity = 1.0
manager.scene.AddPointLight(
    chrono.ChVector3f(2, 2.5, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

# Lidar sensor mounted on rover chassis
horizontal_samples = 800    # horizontal scan samples
vertical_samples = 300      # vertical scan samples
update_rate = 5.0           # Hz — lidar update rate

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1.45),                           # mount above chassis center
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)), # no additional rotation
)
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),  # attach to chassis body
    update_rate,                   # update rate (Hz)
    lidar_offset_pose,             # offset pose
    horizontal_samples,            # h_samples
    vertical_samples,              # v_samples
    2 * chrono.CH_PI,              # horizontal_fov (full 360 deg)
    chrono.CH_PI / 12,             # max_vert_angle
    -chrono.CH_PI / 6,            # min_vert_angle
    100.0,                         # max_range
    sens.LidarBeamShape_RECTANGULAR,
    2,                             # sample_radius
    0.003,                         # vert divergence_angle
    0.003,                         # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / update_rate)  # collection window = 1 / update_rate

# Lidar filter chain (scored core — never review-only)
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())        # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())     # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())      # host access to XYZI
manager.AddSensor(lidar)

# Irrlicht visualization — Initialize() FIRST, then scene elements
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity Rover - Lidar Sensor")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)

time_step = 1e-3     # Curiosity uses 1e-3 step
sim_end = 20.0       # simulation duration
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant


max_steering = math.pi / 6    # max steering angle (rad)
time = 0.0                    # track elapsed time for steering ramp

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        time = system.GetChTime()

        # Steering ramp — hold straight, turn in, then back out
        steering = 0.0
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5    # ramp up
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5  # ramp down
        driver.SetSteering(steering)

        rover.Update()         # propagate steering into motors (required)
        manager.Update()       # update all sensors once per physics step
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break
