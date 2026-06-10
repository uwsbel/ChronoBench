import os
import pychrono as chrono
import pychrono.robot as robot
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

# --- system: Curiosity uses NSC + Bullet collision, Z-up gravity ---
system = chrono.ChSystemNSC()                                          # NSC system for the rover
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)   # required for rover<->ground contact
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))   # Z-up gravity
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)           # contact envelope
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)             # contact margin

# --- rigid ground: fixed box, top surface at z=0 ---
ground_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the ground
ground = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, ground_mat)  # 20x20x1 ground box
ground.SetPos(chrono.ChVector3d(0, 0, -0.5))                          # top surface at z=0
ground.SetFixed(True)                                                 # ground is static
ground.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))  # textured ground
system.Add(ground)                                                    # add ground

# --- rover: pass system to ctor; SetDriver BEFORE Initialize ---
rover = robot.Curiosity(system)                                       # built-in Curiosity rover (owns its bodies)
driver = robot.CuriosityDCMotorControl()                              # DC-motor control driver
rover.SetDriver(driver)                                               # attach driver BEFORE Initialize
init_pos = chrono.ChVector3d(0, 0.2, 0)                               # spawn just above the ground
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                           # identity orientation (w,x,y,z)
rover.Initialize(chrono.ChFramed(init_pos, init_rot))                 # build rover at placement frame

# --- sensor manager + chassis-mounted lidar ---
manager = sens.ChSensorManager(system)                                # oversee all sensors (before adding any)

offset_pose = chrono.ChFramed(chrono.ChVector3d(0.5, 0, 1.0),         # lidar offset on the chassis
                              chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
horizontal_samples = 800                                              # lidar horizontal beam count
vertical_samples = 300                                               # lidar vertical beam count
lidar = sens.ChLidarSensor(
    rover.GetChassis().GetBody(),                                     # attach to the rover chassis
    5.0,                                                             # update_rate (Hz)
    offset_pose,                                                     # offset pose on the chassis
    horizontal_samples,                                              # horizontal samples
    vertical_samples,                                                # vertical samples
    2 * chrono.CH_PI,                                                # horizontal field of view (rad)
    chrono.CH_PI / 12,                                              # max vertical angle
    -chrono.CH_PI / 6,                                              # min vertical angle
    100.0,                                                          # max range
    sens.LidarBeamShape_RECTANGULAR,                                 # beam shape
    2,                                                              # sample radius
    0.003,                                                          # vertical divergence angle
    0.003,                                                          # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                           # return mode
)
lidar.SetName("Lidar Sensor")                                        # name the sensor
lidar.SetLag(0)                                                      # no measurement lag
lidar.SetCollectionWindow(1.0 / 5.0)                                 # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # host access to XYZI
manager.AddSensor(lidar)                                              # register the lidar

# --- Irrlicht visualization (Initialize FIRST, scene elements AFTER) ---
vis = chronoirr.ChVisualSystemIrrlicht()                              # Irrlicht visual system
vis.AttachSystem(system)                                              # bind the physical system
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)                     # Z-up camera convention
vis.SetWindowSize(1280, 720)                                          # window resolution
vis.SetWindowTitle("Curiosity rover - Rigid terrain")                 # window title
vis.Initialize()                                                      # create the window FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))      # PyChrono logo overlay
vis.AddSkyBox()                                                       # sky box backdrop
vis.AddCamera(chrono.ChVector3d(0, 3, 3), chrono.ChVector3d(0, 0, 0))  # camera eye + target
vis.AddTypicalLights()                                                # standard lighting
vis.AddLightWithShadow(chrono.ChVector3d(1.5, -2.5, 5.5), chrono.ChVector3d(0, 0, 0),
                       3, 4, 10, 40, 512)                             # pos, aim, radius, near, far, angle, res
# vis.EnableShadows()   # truth keeps this COMMENTED; no-arg form (item=0) raises TypeError in 9.0.0

# --- simulation parameters ---
time_step = 1e-3                                                      # integration step
render_fps = 50.0                                                     # target playback fps
render_every = max(1, round(1.0 / (render_fps * time_step)))          # render-cadence constant (untagged)
time = 0.0                                                            # simulation clock

# --- main loop: steering input, sensor update, render ---
while vis.Run():                                                      # plain real-time loop (no time bound)
    time = system.GetChTime()                                         # current sim time

    steering = 0.0                                                    # straight until t=1
    if time > 1:                                                      # then ramp steering input over time
        steering = (time - 1) * 0.2                                   # gradually increasing steering
    driver.SetSteering(steering)                                      # set real-time steering input
    rover.Update()                                                    # propagate command into rover motors

    vis.BeginScene(); vis.Render(); vis.EndScene()                    # draw one frame
    for _ in range(render_every):                                     # advance physics between frames
        manager.Update()                                             # pump the lidar once per physics step
        system.DoStepDynamics(time_step)                              # step the dynamics
