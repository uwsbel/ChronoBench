import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

time_step = 1e-3                                                       # integration step (s)
tire_step_size = 1e-3                                                  # tire model substep (s)
sim_end = 20.0                                                         # total simulation time (s)

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # vehicle spawn position
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                          # vehicle spawn orientation

# --- ARTcar vehicle (replaces a bare ChSystem; the vehicle owns the main system) ---
car = veh.ARTcar()                                                    # ARTcar catalog wrapper
car.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision mesh
car.SetChassisFixed(False)                                            # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # set spawn pose
car.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tire model
car.SetTireStepSize(tire_step_size)                                  # tire integration substep
car.Initialize()                                                     # build the vehicle + its system

car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)  # chassis primitives
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
car.SetWheelVisualizationType(veh.VisualizationType_MESH)         # wheels as meshes
car.SetTireVisualizationType(veh.VisualizationType_MESH)          # tires as meshes

system = car.GetSystem()                                             # the vehicle-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact, after Initialize
print("VEHICLE MASS: ", car.GetVehicle().GetMass())                  # report total vehicle mass

# --- rigid terrain (flat patch with texture + color) ---
terrain = veh.RigidTerrain(system)                                   # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material for the patch
patch_mat.SetFriction(0.9)                                           # ground friction coefficient
patch_mat.SetRestitution(0.01)                                       # near-inelastic ground
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # tiled ground texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.8))                       # ground tint
terrain.Initialize()                                                 # finalize terrain

# --- interactive driver bound to the vehicle visual system ---
render_step_size = 1.0 / 50.0                                        # render cadence period (s)

# --- vehicle-specific Irrlicht visual system ---
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # wheeled-vehicle Irrlicht window
vis.SetWindowTitle("ARTcar Lidar Demo")                              # window title
vis.SetWindowSize(1280, 720)                                         # window resolution
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)          # chase camera on the chassis
vis.Initialize()                                                    # build the Irrlicht device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # PyChrono logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # vehicle truths use a directional light
vis.AttachVehicle(car.GetVehicle())                                # bind chassis/wheel visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver on the vis
driver.SetSteeringDelta(render_step_size / 1.0)                    # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                   # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                    # braking rate
driver.Initialize()                                               # finalize driver

chassis_body = car.GetChassisBody()                               # chassis body to ride sensors on

# --- sensor manager + scene lighting (camera-only point/area lights) ---
manager = sens.ChSensorManager(system)                            # oversee all sensors
intensity = 1.0                                                   # light intensity
manager.scene.AddPointLight(                                      # point light #1
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddPointLight(                                      # point light #2
    chrono.ChVector3f(50, 50, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)
manager.scene.AddAreaLight(                                       # one soft area light
    chrono.ChVector3f(0, 0, 20),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
    chrono.ChVector3f(1, 0, 0),
    chrono.ChVector3f(0, -1, 0),
)

# lidar geometry parameters (shared by 3D and 2D lidar)
update_rate = 5.0                                                # lidar update rate (Hz)
horizontal_samples = 800                                         # horizontal beam samples
vertical_samples = 300                                           # vertical beam samples (3D lidar)
horizontal_fov = 2 * chrono.CH_PI                                # full 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                              # upper vertical bound (rad)
min_vert_angle = -chrono.CH_PI / 6                             # lower vertical bound (rad)
lidar_max_range = 100.0                                         # maximum measured range (m)
sample_radius = 2                                              # multi-sample radius
divergence_angle = 0.003                                       # beam divergence (rad)

# lidar offset pose on the chassis (forward + up of the vehicle)
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),                              # forward 1 m, up 1 m on the chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),  # no extra tilt
)

# --- 3D lidar attached to the vehicle chassis ---
lidar = sens.ChLidarSensor(
    chassis_body,                                              # attached to the chassis
    update_rate,                                              # update rate (Hz)
    offset_pose,                                             # offset pose on the chassis
    horizontal_samples,                                     # horizontal samples
    vertical_samples,                                      # vertical samples (3D)
    horizontal_fov,                                       # 360 deg horizontal FOV
    max_vert_angle,                                       # upper vertical angle
    min_vert_angle,                                       # lower vertical angle
    lidar_max_range,                                     # maximum range
    sens.LidarBeamShape_RECTANGULAR,                     # rectangular beam
    sample_radius,                                       # sample radius
    divergence_angle,                                    # vertical divergence
    divergence_angle,                                    # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,               # strongest-return mode
)
lidar.SetName("3D Lidar Sensor")                          # sensor name
lidar.SetLag(0)                                           # no lag
lidar.SetCollectionWindow(1.0 / update_rate)            # collection window = 1 / update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())               # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())            # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())             # host access to XYZI
manager.AddSensor(lidar)                                 # register the 3D lidar

# --- 2D lidar attached to the vehicle chassis (single scan plane) ---
lidar_2d = sens.ChLidarSensor(
    chassis_body,                                         # attached to the chassis
    update_rate,                                         # update rate (Hz)
    offset_pose,                                        # same offset pose on the chassis
    horizontal_samples,                                # horizontal samples
    1,                                                 # vertical samples = 1 -> 2D scan
    horizontal_fov,                                    # 360 deg horizontal FOV
    0.0,                                              # max vertical angle = 0 (planar)
    0.0,                                              # min vertical angle = 0 (planar)
    lidar_max_range,                                  # maximum range
    sens.LidarBeamShape_RECTANGULAR,                  # rectangular beam
    sample_radius,                                    # sample radius
    divergence_angle,                                 # vertical divergence
    divergence_angle,                                 # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,            # strongest-return mode
)
lidar_2d.SetName("2D Lidar Sensor")                    # sensor name
lidar_2d.SetLag(0)                                     # no lag
lidar_2d.SetCollectionWindow(1.0 / update_rate)      # collection window = 1 / update_rate
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth"))  # 2D depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())          # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())       # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # preview
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())        # host access to XYZI
manager.AddSensor(lidar_2d)                            # register the 2D lidar

# --- third-person view camera sensor attached to the chassis ---
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-5.0, 0, 2.0),                  # behind and above the chassis (third-person)
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),  # slight downward tilt
)
cam = sens.ChCameraSensor(
    chassis_body,                                     # rides on the chassis (third-person)
    30,                                              # physical update rate (Hz)
    cam_offset_pose,                                 # offset pose on the chassis
    1280, 720,                                       # image width, height
    1.408,                                          # horizontal FOV (rad)
)
cam.SetName("Third Person Camera")                   # sensor name
cam.SetLag(0)                                        # no lag
cam.SetCollectionWindow(0)                           # instantaneous exposure
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))  # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())           # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/third_person/"))  # save RGB frames
manager.AddSensor(cam)                               # register the camera

# --- render cadence + recording scaffolding ---
render_fps = 50.0                                                     # review video fps target
render_every = max(1, round(1.0 / (render_fps * time_step)))         # untagged cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                       # spin in place to match wall-clock
while vis.Run() and system.GetChTime() < sim_end:
    vis.BeginScene()                                                # begin Irrlicht frame
    vis.Render()                                                    # draw the scene
    vis.EndScene()                                                  # end Irrlicht frame
    for _ in range(render_every):
        sim_time = system.GetChTime()                              # current sim time
        driver_inputs = driver.GetInputs()                         # current driver inputs

        driver.Synchronize(sim_time)                               # sync driver
        terrain.Synchronize(sim_time)                              # sync terrain
        car.Synchronize(sim_time, driver_inputs, terrain)          # sync vehicle with inputs + terrain
        vis.Synchronize(sim_time, driver_inputs)                   # sync visual HUD

        driver.Advance(time_step)                                  # advance driver
        terrain.Advance(time_step)                                 # advance terrain
        car.Advance(time_step)                                     # advance vehicle (steps the system)
        vis.Advance(time_step)                                     # advance vis

        manager.Update()                                           # pump all sensors once per step

        # sensor buffer access — the truth-style sensor data output
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()              # 3D lidar point cloud
        if xyzi_buffer.HasData():
            print('3D Lidar buffer. Points: {0}'.format(xyzi_buffer.Width * xyzi_buffer.Height))
        xyzi_2d = lidar_2d.GetMostRecentXYZIBuffer()               # 2D lidar point cloud
        if xyzi_2d.HasData():
            print('2D Lidar buffer. Points: {0}'.format(xyzi_2d.Width * xyzi_2d.Height))
        rgba_buffer = cam.GetMostRecentRGBA8Buffer()               # camera RGBA frame
        if rgba_buffer.HasData():
            print('Camera buffer. Resolution: {0}x{1}'.format(rgba_buffer.Width, rgba_buffer.Height))

        realtime_timer.Spin(time_step)                             # match wall-clock to sim time
        if system.GetChTime() >= sim_end:
            break
