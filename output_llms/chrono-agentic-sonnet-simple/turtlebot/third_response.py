import os
import math
import random
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# System setup
system = chrono.ChSystemNSC()
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # contact scene requires bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))    # Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)            # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)              # collision margin

# Ground (rigid) - adjusted position to z=-0.6
ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))                           # top surface at z=0.1
ground.SetFixed(True)
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

# Add 5 randomly-placed boxes for TurtleBot to interact with
random.seed(42)                                                        # reproducible placement
box_mat = chrono.ChContactMaterialNSC()
for i in range(5):
    bx = random.uniform(-3, 3)                                         # random x in [-3,3]
    by = random.uniform(-3, 3)                                         # random y in [-3,3]
    box = chrono.ChBodyEasyBox(0.3, 0.3, 0.3, 100, True, True, box_mat)
    box.SetPos(chrono.ChVector3d(bx, by, 0.25))                        # sit on ground surface
    system.Add(box)

# TurtleBot setup - pose in constructor
init_pos = chrono.ChVector3d(0, 0, 0.2)                               # start above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation
robot_tb = robot.TurtleBot(system, init_pos, init_rot)                 # pose in constructor
robot_tb.Initialize()                                                  # no-arg initialize

# Find chassis body by name for sensor mounting
chassis_body = None
for b in system.GetBodies():
    if b.GetName() == "chassis_body":
        chassis_body = b
        break

LEFT_DRIVE_WHEEL  = 0                                                  # WheelID: 0=LEFT
RIGHT_DRIVE_WHEEL = 1                                                  # WheelID: 1=RIGHT

# Motion control function for TurtleBot (straight/left/right modes)
def move(mode):
    if mode == 'straight':
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)             # forward left
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)            # forward right
    elif mode == 'left':
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)                    # stop left wheel
        robot_tb.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)            # drive right
    elif mode == 'right':
        robot_tb.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)             # drive left
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)                   # stop right wheel

# Sensor manager
manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(1, 1, 1), 500.0)           # main point light
manager.scene.AddPointLight(chrono.ChVector3f(-2, -2.5, 100),
                            chrono.ChColor(1, 1, 1), 500.0)           # secondary point light

# Lidar sensor mounted on TurtleBot chassis
lidar_offset = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 0.4),                                     # above chassis center
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1)),
)
horizontal_samples = 800                                               # horizontal resolution
vertical_samples   = 1                                                 # 2D lidar (flat scan)
lidar = sens.ChLidarSensor(
    chassis_body,                                                      # attach to chassis body
    5.0,                                                               # update_rate (Hz)
    lidar_offset,                                                      # offset pose
    horizontal_samples,                                                # h_samples
    vertical_samples,                                                  # v_samples (1=2D)
    2 * chrono.CH_PI,                                                  # horizontal_fov
    0,                                                                 # max_vert_angle (0 for 2D)
    0,                                                                 # min_vert_angle (0 for 2D)
    100.0,                                                             # max_range
    sens.LidarBeamShape_RECTANGULAR,                                   # beam shape
    2,                                                                 # sample_radius
    0.003,                                                             # vert divergence
    0.003,                                                             # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)
lidar.SetCollectionWindow(1.0 / 5.0)                                   # collection = 1/update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())                              # depth+intensity access
lidar.PushFilter(sens.ChFilterPCfromDepth())                           # depth -> point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                            # XYZI host access
manager.AddSensor(lidar)

# Irrlicht visualization block — Initialize() FIRST, scene elements AFTER
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                      # Z-up convention
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Lidar + Random Boxes")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -4, 3), chrono.ChVector3d(0, 0, 0.5))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                             # shadow light

# Simulation parameters
time_step  = 2e-3                                                      # TurtleBot uses 2e-3
sim_end    = 20.0                                                      # total simulation time
render_fps = 50.0
render_every = max(1, round(1.0 / (render_fps * time_step)))          # untagged cadence constant


# Main simulation loop — move straight using the move() control function
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene(); vis.Render(); vis.EndScene()
    for _ in range(render_every):
        move('straight')                                               # drive TurtleBot forward
        manager.Update()                                               # update sensor manager
        system.DoStepDynamics(time_step)                               # advance physics
        if system.GetChTime() >= sim_end:
            break
