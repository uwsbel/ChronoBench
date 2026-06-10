import os
import math
import random
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

time_step = 2e-3                                          # TurtleBot integrator step
sim_end = 12.0                                            # total simulated time

system = chrono.ChSystemNSC()                             # rovers use NSC
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # contact needs Bullet
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # g, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # small rover envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small rover margin

ground_mat = chrono.ChContactMaterialNSC()               # NSC contact material for ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)   # 20x20x1 floor
ground.SetPos(chrono.ChVector3d(0, 0, -0.6))             # box top at z = -0.1
ground.SetFixed(True)                                    # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete look
system.Add(ground)                                       # add ground to the system

random.seed(42)                                          # deterministic box layout
box_mat = chrono.ChContactMaterialNSC()                  # NSC material for the boxes
for i in range(5):                                       # 5 randomly placed interaction boxes
    bx = random.uniform(-4.0, 4.0)                       # random X within the floor
    by = random.uniform(-4.0, 4.0)                       # random Y within the floor
    box = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 250, True, True, box_mat)   # 0.4 m cube
    box.SetPos(chrono.ChVector3d(bx, by, 0.1))           # rest on the floor (top at -0.1)
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.6, 0.2, 0.2))   # reddish boxes
    system.Add(box)                                      # add the box to the system

init_pos = chrono.ChVector3d(0, 0.2, 0)                  # TurtleBot spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)             # identity orientation (w,x,y,z)
robot_tb = robot.TurtleBot(system, init_pos, init_rot)   # pose passed in the constructor
robot_tb.Initialize()                                    # builds bodies/wheels/motors, no-arg

LEFT_DRIVE_WHEEL = 0                                      # WheelID 0 = LEFT
RIGHT_DRIVE_WHEEL = 1                                     # WheelID 1 = RIGHT
wheel_speed = 2 * math.pi                                 # nominal wheel angular speed (rad/s)

def move(mode):                                          # differential-drive command helper
    if mode == 'straight':                               # both wheels forward at equal speed
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'left':                                 # pivot left: stop left, drive right
        robot_tb.SetMotorSpeed(0, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(-wheel_speed, RIGHT_DRIVE_WHEEL)
    elif mode == 'right':                                # pivot right: drive left, stop right
        robot_tb.SetMotorSpeed(-wheel_speed, LEFT_DRIVE_WHEEL)
        robot_tb.SetMotorSpeed(0, RIGHT_DRIVE_WHEEL)

chassis_body = next(b for b in system.GetBodies()        # TurtleBot chassis: lidar mount body
                    if b.GetName() == "chassis_body")

manager = sens.ChSensorManager(system)                   # oversees all sensors
intensity = 1.0                                          # uniform light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)   # fill light
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)   # second light

horizontal_samples = 800                                 # lidar horizontal beams
vertical_samples = 300                                   # lidar vertical channels
update_rate = 5.0                                        # lidar physical update rate (Hz)
offset_pose = chrono.ChFramed(                           # lidar offset on the chassis
    chrono.ChVector3d(0, 0, 0.3),                       # mounted above the chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),   # no tilt
)
lidar = sens.ChLidarSensor(
    chassis_body,                                        # body the lidar is attached to
    update_rate,                                        # update rate (Hz)
    offset_pose,                                        # offset pose on the chassis
    horizontal_samples,                                 # horizontal samples
    vertical_samples,                                   # vertical samples
    2 * chrono.CH_PI,                                  # horizontal FOV (full 360 deg)
    chrono.CH_PI / 12,                                 # max vertical angle
    -chrono.CH_PI / 6,                                 # min vertical angle
    100.0,                                             # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                   # rectangular beam
    2,                                                 # sample radius
    0.003,                                             # vertical divergence angle
    0.003,                                             # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,             # return mode
)
lidar.SetName("Lidar Sensor")                            # sensor name
lidar.SetLag(0)                                          # no lag
lidar.SetCollectionWindow(1.0 / update_rate)            # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())               # host access to depth + intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())            # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())             # host access to XYZI
manager.AddSensor(lidar)                                 # register the lidar

vis = chronoirr.ChVisualSystemIrrlicht()                 # Irrlicht review window
vis.AttachSystem(system)                                 # bind the system's visuals
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)       # Z-up camera convention
vis.SetWindowSize(1280, 720)                             # window resolution
vis.SetWindowTitle("TurtleBot - Lidar scanning")        # window title
vis.Initialize()                                         # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))   # corner logo
vis.AddSkyBox()                                          # sky backdrop
vis.AddCamera(chrono.ChVector3d(0, 2.5, 1.5), chrono.ChVector3d(0, 0, 1))   # eye, target
vis.AddTypicalLights()                                   # standard lights
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                # shadow-casting light

render_fps = 50.0                                        # review frame rate
render_every = max(1, round(1.0 / (render_fps * time_step)))   # untagged cadence constant

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        move('straight')                                 # drive the TurtleBot forward
        manager.Update()                                 # pump sensors once per physics step
        system.DoStepDynamics(time_step)                 # advance one physics step
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()    # most recent point-cloud buffer
        if xyzi_buffer.HasData():                        # only read once the sensor has ticked
            xyzi = xyzi_buffer.GetXYZIData()             # host XYZI point-cloud data
            print('Lidar buffer received. Points: {0}x{1}'.format(xyzi_buffer.Width,
                                                                  xyzi_buffer.Height))
        if system.GetChTime() >= sim_end:
            break
