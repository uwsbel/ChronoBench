import os
import math
import random
import pychrono as chrono
import pychrono.robot as turtlebot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

system = chrono.ChSystemNSC()                                         # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # Bullet collision (rover<->ground contact)
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up world, g downward
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # collision envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # collision margin

ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for terrain
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # rigid terrain box
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))                          # top surface at z=-0.1
ground.SetFixed(True)                                                 # static terrain
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)

box_mat = chrono.ChContactMaterialNSC()                               # contact material for the obstacle boxes
for i in range(5):                                                    # 5 randomly placed boxes for interaction
    bx = random.uniform(-3.0, 3.0)                                    # random x in the playfield
    by = random.uniform(-3.0, 3.0)                                    # random y in the playfield
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 1000, True, True, box_mat)  # small dynamic box
    box.SetPos(chrono.ChVector3d(bx, by, 0.2))                        # rest on the terrain
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.3, 0.8))     # blue boxes
    system.Add(box)

init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above terrain
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation
robot = turtlebot.TurtleBot(system, init_pos, init_rot)               # TurtleBot owns its bodies; pose in ctor
robot.Initialize()                                                    # no-arg Initialize for TurtleBot

chassis_body = None                                                   # locate the chassis to mount the lidar
for b in system.GetBodies():                                          # search the robot's own bodies
    if b.GetName() == "chassis_body":
        chassis_body = b
        break

LEFT_DRIVE_WHEEL = 0                                                  # WheelID: 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                                 #          1 = RIGHT

def move(mode):                                                       # differential-drive controller
    if mode == "straight":                                           # both wheels drive forward
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "left":                                            # left wheel stops -> pivot left
        robot.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(-math.pi, RIGHT_DRIVE_WHEEL)
    elif mode == "right":                                          # right wheel stops -> pivot right
        robot.SetMotorSpeed(-math.pi, LEFT_DRIVE_WHEEL)
        robot.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)
    else:                                                           # guard invalid mode
        raise ValueError("Invalid mode: " + str(mode))

manager = sens.ChSensorManager(system)                                # sensor manager oversees the lidar

horizontal_samples = 800                                              # lidar horizontal beam count
vertical_samples = 300                                                # lidar vertical beam count
offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0.3),           # mount above the chassis
                              chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
lidar = sens.ChLidarSensor(
    chassis_body,                          # body the lidar is attached to
    5.0,                                   # update_rate (Hz)
    offset_pose,                           # offset pose
    horizontal_samples,                    # h_samples
    vertical_samples,                      # v_samples
    2 * chrono.CH_PI,                      # horizontal_fov (rad)
    chrono.CH_PI / 12,                     # max_vert_angle
    -chrono.CH_PI / 6,                     # min_vert_angle
    100.0,                                 # max_range
    sens.LidarBeamShape_RECTANGULAR,       # beam shape
    2,                                     # sample_radius
    0.003,                                 # vert divergence_angle
    0.003,                                 # hori divergence_angle
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar.SetName("Lidar Sensor")                                         # sensor name
lidar.SetLag(0)                                                       # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                                  # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # host access to XYZI
manager.AddSensor(lidar)                                              # register the lidar

vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht real-time window
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up camera
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("TurtleBot robot - Rigid terrain")
vis.Initialize()                                                      # Initialize FIRST, then scene elements
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 1.5, 0.2), chrono.ChVector3d(0, 0, 0.2))  # follow camera
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                             # pos, aim, radius, near, far, angle, res

time_step = 2e-3                                                      # TurtleBot uses 2e-3
render_fps = 50.0                                                     # review video frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))         # render cadence constant


move("straight")                                                      # drive the TurtleBot straight
time = 0                                                              # simulation clock
while vis.Run():                                                      # plain real-time loop
    vis.BeginScene(); vis.Render(); vis.EndScene()                   # draw one frame
    for _ in range(render_every):                                    # advance physics between frames
        manager.Update()                                            # pump the lidar once per step
        time += time_step                                           # advance clock
        system.DoStepDynamics(time_step)                           # step dynamics (no robot.Update for TurtleBot)
