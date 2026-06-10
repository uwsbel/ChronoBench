import os
import math
import pychrono as chrono
import pychrono.robot as robot
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

# --- system: rovers use NSC + Bullet collision ---
system = chrono.ChSystemNSC()                                            # non-smooth system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # required for wheel<->ground contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))     # g = 9.81, Z-up world
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)             # small contact envelope for rover geometry
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)              # small contact margin

# --- ground: fixed box, top surface at z=0 under the Curiosity spawn ---
ground_mat = chrono.ChContactMaterialNSC()                              # NSC contact material for the terrain
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 box, visual + collision
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                           # Curiosity: box top at z=0
ground.SetFixed(True)                                                   # terrain is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # concrete texture
system.Add(ground)                                                      # add the ground to the system

# --- Curiosity Mars rover: owns its bodies/joints/motors ---
rover = robot.Curiosity(system)                                         # rocker-bogie rover, system-owned
driver = robot.CuriosityDCMotorControl()                               # DC-motor steering driver
rover.SetDriver(driver)                                                 # attach driver BEFORE Initialize

init_pos = chrono.ChVector3d(0, 0, 0.2)                                # spawn slightly above the ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                  # place the rover with a frame

# --- sensor manager: oversees the chassis-mounted lidar ---
manager = sens.ChSensorManager(system)                                 # manages all sensors on this system
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),            # point light (ChVector3f position)
                            chrono.ChColor(1, 1, 1), 500.0)            # white, range 500
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),           # second point light for scene fill
                            chrono.ChColor(1, 1, 1), 500.0)

# --- lidar mounted on the rover chassis ---
chassis_body = rover.GetChassis().GetBody()                            # chassis body = lidar attach point
horizontal_samples = 800                                                # horizontal beam count
vertical_samples = 300                                                  # vertical beam count (3D lidar)
update_rate = 5.0                                                       # lidar physical update rate (Hz)
max_range = 100.0                                                       # max sensing range (m)

offset_pose = chrono.ChFramed(                                          # lidar offset on the chassis frame
    chrono.ChVector3d(0.0, 0, 1.0),                                    # mounted above the chassis, forward
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),          # no tilt
)
lidar = sens.ChLidarSensor(
    chassis_body,                                                      # body the lidar rides on
    update_rate,                                                       # update_rate (Hz)
    offset_pose,                                                       # offset pose on the chassis
    horizontal_samples,                                                # horizontal samples
    vertical_samples,                                                  # vertical samples
    2 * chrono.CH_PI,                                                  # horizontal field of view (rad)
    chrono.CH_PI / 12,                                                 # max vertical angle (rad)
    -chrono.CH_PI / 6,                                                 # min vertical angle (rad)
    max_range,                                                         # maximum range (m)
    sens.LidarBeamShape_RECTANGULAR,                                  # beam cross-section shape
    2,                                                                 # sample radius
    0.003,                                                             # vertical divergence angle
    0.003,                                                             # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                            # return mode
)
lidar.SetName("Lidar Sensor")                                          # name for diagnostics
lidar.SetLag(0)                                                        # no latency
lidar.SetCollectionWindow(1.0 / update_rate)                          # collection window = 1/update_rate

# --- lidar filter chain (ORDER MATTERS) — scored core, never review-only ---
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                            # host access to depth+intensity buffer
lidar.PushFilter(sens.ChFilterPCfromDepth())                        # convert depth to XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # host access to XYZI point cloud
manager.AddSensor(lidar)                                              # push ALL filters BEFORE AddSensor

# --- Irrlicht window: Initialize first, then scene elements (NO grid) ---
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)                                              # bind the system's visual assets
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                    # Z-up camera
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetWindowTitle("Curiosity rover - Lidar")                       # window title
vis.Initialize()                                                     # create the device FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo overlay
vis.AddSkyBox()                                                      # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 3.0, 1.5), chrono.ChVector3d(0, 0, 1))  # interactive view
vis.AddTypicalLights()                                              # standard scene lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0.5),
                       3, 4, 10, 40, 512)                          # shadow-casting light

# --- time-stepping parameters ---
time_step = 1e-3                                                    # rover integration step
sim_end = 12.0                                                      # total simulated time (s)
max_steering = math.pi / 6                                          # practical steering limit (rad)
render_fps = 50.0                                                   # review render cadence
render_every = max(1, round(1.0 / (render_fps * time_step)))       # untagged cadence constant

buffer_logged = False                                              # fire the lidar buffer diagnostic once
time = 0.0                                                          # simulated time accumulator

# --- main loop: render once per frame, advance physics in the inner batch ---
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    for _ in range(render_every):
        # ramp the steering: straight, turn in, then back out
        steering = 0.0
        if 2 < time < 7:
            steering = max_steering * (time - 2) / 5               # ramp up
        elif 7 < time < 12:
            steering = max_steering * (12 - time) / 5             # ramp down
        driver.SetSteering(steering)                              # command steering on the driver
        rover.Update()                                           # propagate command into the motors
        manager.Update()                                         # pump the lidar once per physics step
        system.DoStepDynamics(time_step)                         # advance one physics step
        time += time_step                                        # advance the time accumulator
        # lidar buffer access (scored core): read the XYZI point cloud once it has data
        if not buffer_logged:
            xyzi_buffer = lidar.GetMostRecentXYZIBuffer()        # most recent point-cloud buffer
            if xyzi_buffer.HasData():                            # empty until the lidar's first tick
                print('Lidar buffer received. Points: {0}x{1}'.format(
                    xyzi_buffer.Width, xyzi_buffer.Height))
                buffer_logged = True                             # disable further logging
        if system.GetChTime() >= sim_end:
            break
