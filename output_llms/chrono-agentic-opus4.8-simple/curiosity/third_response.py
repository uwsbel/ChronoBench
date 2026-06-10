import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

system = chrono.ChSystemNSC()                                        # NSC system (rovers use NSC)
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system required for wheel-terrain contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # g = 9.81, Z-down
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)          # small contact envelope for rover geometry
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)            # small contact margin

ground_mat = chrono.ChContactMaterialNSC()                          # NSC contact material for the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20 box, 1 m thick
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                        # Curiosity: top of box at z=0
ground.SetFixed(True)                                               # ground is fixed
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # ground texture
system.Add(ground)                                                  # add ground to system

rover = robot.Curiosity(system)                                     # built-in Curiosity Mars rover, owns its bodies
driver = robot.CuriosityDCMotorControl()                           # DC-motor steering driver
rover.SetDriver(driver)                                             # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(0, 0.2, 0)                             # rover spawn position (wheels rest on ground top)
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                        # identity rotation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))              # Initialize takes a ChFramed

manager = sens.ChSensorManager(system)                             # sensor manager oversees the lidar

offset_pose = chrono.ChFramed(                                     # lidar mount pose on the chassis
    chrono.ChVector3d(0.0, 0, 1.0),                               # forward-of-center, above chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),      # no tilt
)
horizontal_samples = 800                                           # lidar horizontal beams
vertical_samples = 300                                             # lidar vertical beams
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),                                 # attach to the rover chassis
    5.0,                                                          # update_rate (Hz)
    offset_pose,                                                  # offset pose
    horizontal_samples,                                           # h_samples
    vertical_samples,                                             # v_samples
    2 * chrono.CH_PI,                                            # horizontal_fov (rad)
    chrono.CH_PI / 12,                                          # max_vert_angle
    -chrono.CH_PI / 6,                                          # min_vert_angle
    100.0,                                                       # max_range
    sens.LidarBeamShape_RECTANGULAR,                            # beam shape
    2,                                                          # sample_radius
    0.003,                                                      # vert divergence angle
    0.003,                                                      # hori divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                     # return mode
)
lidar.SetName("Lidar Sensor")                                    # lidar name
lidar.SetLag(0)                                                  # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                            # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                       # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                     # host access to XYZI
manager.AddSensor(lidar)                                        # register the lidar with the manager

vis = chronoirr.ChVisualSystemIrrlicht()                       # Irrlicht review window
vis.AttachSystem(system)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)              # Z-up
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle("Curiosity rover - Lidar")
vis.Initialize()                                              # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 1))
vis.AddTypicalLights()
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                     # pos, aim, radius, near, far, angle, resolution

time_step = 1e-3                                              # integration step
sim_end = 15.0                                               # simulation duration
max_steering = math.pi / 6                                  # steering limit
render_fps = 50.0                                           # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))  # untagged cadence constant
time = 0.0                                                   # sim time tracker
log_info = True                                              # fire the lidar log once

while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        steering = 0.0                                       # straight by default
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5         # ramp steering in
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5        # ramp steering out
        driver.SetSteering(steering)                         # command steering on the driver
        rover.Update()                                       # propagate command into the rover motors
        manager.Update()                                     # pump the lidar once per physics step
        buf = lidar.GetMostRecentXYZIBuffer()                # most recent point-cloud buffer
        if log_info and buf.HasData():                       # report once after the first lidar tick
            print("Lidar buffer. Points: ", buf.Width * buf.Height)
            log_info = False                                 # disable further logging
        system.DoStepDynamics(time_step)                     # advance one step
        time += time_step                                    # track sim time
        if system.GetChTime() >= sim_end:
            break
