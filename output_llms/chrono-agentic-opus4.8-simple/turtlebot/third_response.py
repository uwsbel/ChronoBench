import os
import math
import random
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

random.seed(0)                                                       # deterministic box layout

system = chrono.ChSystemNSC()                                        # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET) # contact -> Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81)) # g = 9.81 down (Z-up)
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)         # small rover envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)          # small rover margin

ground_mat = chrono.ChContactMaterialNSC()                          # NSC contact material
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 ground box
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))                        # ground top at z = -0.1
ground.SetFixed(True)                                               # static floor
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
system.Add(ground)

init_pos = chrono.ChVector3d(0, 0.2, 0)                             # TurtleBot spawn above ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                         # identity (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)             # pose in constructor
robot_tb.Initialize()                                              # no-arg initialize
chassis_body = system.SearchBody("chassis_body")                  # rover chassis body

box_mat = chrono.ChContactMaterialNSC()                            # box contact material
for i in range(5):                                                 # 5 randomly placed boxes
    bx = random.uniform(-4, 4)                                     # random x in [-4, 4]
    by = random.uniform(-4, 4)                                     # random y in [-4, 4]
    box = chrono.ChBodyEasyBox(0.3, 0.3, 0.3, 200, True, True, box_mat)  # 0.3 m cube
    box.SetPos(chrono.ChVector3d(bx, by, 0.15))                    # rest on the ground
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))  # reddish boxes
    system.Add(box)

manager = sens.ChSensorManager(system)                             # oversee all sensors

offset_pose = chrono.ChFramed(                                     # lidar mounted on the chassis
    chrono.ChVector3d(0, 0, 0.3),                                  # 0.3 m above chassis origin
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),       # no tilt
)
horizontal_samples = 800                                           # horizontal beams
vertical_samples = 300                                             # vertical beams
lidar = sens.ChLidarSensor(
    chassis_body,                                                # attach to the rover chassis
    5.0,                                                          # update_rate (Hz)
    offset_pose,                                                  # offset pose on the chassis
    horizontal_samples,                                          # h_samples
    vertical_samples,                                            # v_samples
    2 * chrono.CH_PI,                                            # horizontal_fov (rad)
    chrono.CH_PI / 12,                                          # max_vert_angle
    -chrono.CH_PI / 6,                                          # min_vert_angle
    100.0,                                                      # max_range
    sens.LidarBeamShape_RECTANGULAR,                            # beam shape
    2,                                                          # sample_radius
    0.003,                                                      # vertical divergence
    0.003,                                                      # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                      # return mode
)
lidar.SetName("Lidar Sensor")                                     # name the lidar
lidar.SetLag(0)                                                   # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                             # collection window = 1/update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))
lidar.PushFilter(sens.ChFilterDIAccess())                        # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                     # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())                      # host access to XYZI
manager.AddSensor(lidar)                                          # register the lidar

LEFT_DRIVE_WHEEL = 0                                              # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                             # WheelID 1 = RIGHT
wheel_speed = 2 * math.pi                                         # nominal wheel speed (rad/s)


def move(mode):                                                  # differential-drive command
    if mode == "straight":                                       # both wheels forward
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == "left":                                         # pivot left
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == "right":                                        # pivot right
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)


vis = chronoirr.ChVisualSystemIrrlicht()                         # Irrlicht render window
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)               # Z-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot - Rigid terrain with lidar")
vis.Initialize()                                                # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                        # pos, aim, radius, near, far, angle, res

time_step = 2e-3                                                 # TurtleBot uses 2e-3
sim_end = 10.0                                                   # total sim time (s)
render_fps = 50.0                                                # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))    # untagged cadence constant
move("straight")                                                # drive the TurtleBot straight
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        manager.Update()                                        # pump sensors once per step
        system.DoStepDynamics(time_step)
        if system.GetChTime() >= sim_end:
            break
