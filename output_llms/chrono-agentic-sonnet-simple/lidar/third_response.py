import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # locate vehicle data files

step_size = 1e-3                                                     # physics step (s)
sim_end = 20.0                                                       # simulation end time (s)
render_fps = 50.0                                                     # frames per second for rendering

terrain_length = 200.0                                               # terrain X size (m)
terrain_width = 200.0                                                # terrain Y size (m)

init_loc = chrono.ChVector3d(0, 0, 0.5)                             # initial vehicle location
init_rot = chrono.QuatFromAngleZ(0)                                  # initial vehicle rotation

artcar = veh.ARTcar()                                                # ARTcar catalog wrapper
artcar.SetContactMethod(chrono.ChContactMethod_NSC)                  # NSC for rigid terrain
artcar.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision
artcar.SetChassisFixed(False)                                        # chassis must be free
artcar.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))      # spawn position + orientation
artcar.SetTireType(veh.TireModelType_RIGID)                          # rigid tire for rigid terrain
artcar.SetTireStepSize(step_size)                                    # tire integration step
artcar.Initialize()                                                   # build the vehicle

system = artcar.GetSystem()                                          # get the vehicle's ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED: Bullet collision

print("VEHICLE MASS: ", artcar.GetVehicle().GetMass())              # truth's literal banner

artcar.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)      # chassis vis
artcar.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)    # suspension vis
artcar.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)      # steering vis
artcar.SetWheelVisualizationType(veh.VisualizationType_MESH)               # wheel vis
artcar.SetTireVisualizationType(veh.VisualizationType_MESH)                # tire vis

terrain = veh.RigidTerrain(artcar.GetSystem())                       # flat rigid terrain

patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # friction coefficient
patch_mat.SetRestitution(0.01)                                       # near-zero bounce

patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, terrain_length, terrain_width)  # flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)            # tile texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                                        # terrain color

terrain.Initialize()                                                 # finalize terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                    # vehicle Irrlicht vis
vis.SetWindowTitle("ARTcar + Lidar Demo")                            # window title
vis.SetWindowSize(1280, 720)                                          # window dimensions
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.75), 9.0, 0.5)        # third-person Irrlicht camera
vis.Initialize()                                                      # Initialize FIRST
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # Chrono logo
vis.AddSkyBox()                                                       # sky box
vis.AddLightDirectional()                                             # directional light (vehicle truth)
vis.AttachVehicle(artcar.GetVehicle())                               # bind vehicle assets

driver = veh.ChInteractiveDriverIRR(vis)                             # interactive driver
driver.SetSteeringDelta(render_fps / 50.0 * 1.0 / 1.0)              # steering ramp
driver.SetThrottleDelta(render_fps / 50.0 * 1.0 / 1.0)              # throttle ramp
driver.SetBrakingDelta(render_fps / 50.0 * 1.0 / 0.3)               # braking ramp
driver.Initialize()                                                   # finalize driver

manager = sens.ChSensorManager(system)                               # sensor manager

intensity = 1.0                                                      # light intensity
manager.scene.AddPointLight(                                         # point light for sensor scene
    chrono.ChVector3f(0, 0, 100),
    chrono.ChColor(intensity, intensity, intensity),
    500.0,
)

lidar_offset_pose = chrono.ChFramed(                                 # lidar offset: from prompt (1.0,0,1)
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)

h_samples = 800                                                      # horizontal samples (3D lidar)
v_samples = 300                                                      # vertical samples (3D lidar)

lidar_3d = sens.ChLidarSensor(                                       # 3D lidar on vehicle chassis
    artcar.GetChassisBody(),                                         # attached to chassis
    5.0,                                                             # update rate (Hz)
    lidar_offset_pose,                                               # offset pose
    h_samples,                                                       # horizontal samples
    v_samples,                                                       # vertical samples
    2 * chrono.CH_PI,                                               # horizontal FOV (full 360)
    chrono.CH_PI / 12,                                               # max vertical angle
    -chrono.CH_PI / 6,                                               # min vertical angle
    100.0,                                                           # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                                 # beam shape
    2,                                                               # sample radius
    0.003,                                                           # vertical divergence angle
    0.003,                                                           # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                           # return mode
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(0)                                                   # no lag
lidar_3d.SetCollectionWindow(1.0 / 5.0)                              # collection window = 1/update_rate

lidar_3d.PushFilter(sens.ChFilterVisualize(h_samples, v_samples, "3D Raw Lidar Depth"))  # depth preview
lidar_3d.PushFilter(sens.ChFilterDIAccess())                         # depth+intensity host access
lidar_3d.PushFilter(sens.ChFilterPCfromDepth())                      # depth -> XYZI point cloud
lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))  # point cloud preview
lidar_3d.PushFilter(sens.ChFilterXYZIAccess())                       # XYZI host access
manager.AddSensor(lidar_3d)                                          # register 3D lidar

lidar_2d = sens.ChLidarSensor(                                       # 2D lidar on vehicle chassis
    artcar.GetChassisBody(),                                         # attached to chassis
    5.0,                                                             # update rate (Hz)
    lidar_offset_pose,                                               # same offset pose
    800,                                                             # horizontal samples (2D)
    1,                                                               # v_samples = 1 for 2D
    2 * chrono.CH_PI,                                               # horizontal FOV (full 360)
    0,                                                               # max_vert_angle = 0 for 2D
    0,                                                               # min_vert_angle = 0 for 2D
    100.0,                                                           # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                                 # beam shape
    2,                                                               # sample radius
    0.003,                                                           # vertical divergence angle
    0.003,                                                           # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                           # return mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)                                                   # no lag
lidar_2d.SetCollectionWindow(1.0 / 5.0)                              # collection window = 1/update_rate

lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "2D Raw Lidar Depth"))   # 2D depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())                         # depth+intensity host access
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                      # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # point cloud preview
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                       # XYZI host access
manager.AddSensor(lidar_2d)                                          # register 2D lidar

cam_offset_pose = chrono.ChFramed(                                   # third-person camera offset
    chrono.ChVector3d(-6, 0, 2),                                     # behind and above chassis
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),       # slight downward tilt
)

cam = sens.ChCameraSensor(                                           # third-person camera on chassis
    artcar.GetChassisBody(),                                         # rides on chassis
    30,                                                              # update rate (Hz) - physical
    cam_offset_pose,                                                  # offset pose
    1280, 720,                                                        # width, height
    1.408,                                                           # horizontal FOV (rad)
)
cam.SetName("Third Person Camera")
cam.SetLag(0)                                                        # no lag
cam.SetCollectionWindow(0)                                           # camera: exposure window = 0

cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                           # RGBA8 host access
cam.PushFilter(sens.ChFilterSave("cam/third_person/"))              # save third-person frames
manager.AddSensor(cam)                                               # register camera

render_every = max(1, round(1.0 / (render_fps * step_size)))        # render cadence constant


realtime_timer = chrono.ChRealtimeStepTimer()                        # real-time pacing

while vis.Run() and artcar.GetSystem().GetChTime() < sim_end:
    sim_time = artcar.GetSystem().GetChTime()                        # current sim time

    vis.BeginScene()                                                 # begin Irrlicht render
    vis.Render()                                                     # render scene
    vis.EndScene()                                                   # end Irrlicht render

    for _ in range(render_every):
        sim_time = artcar.GetSystem().GetChTime()                    # inner loop time

        driver_inputs = driver.GetInputs()                           # get current driver inputs

        driver.Synchronize(sim_time)                                 # sync driver
        terrain.Synchronize(sim_time)                                # sync terrain
        artcar.Synchronize(sim_time, driver_inputs, terrain)         # sync vehicle with driver + terrain
        vis.Synchronize(sim_time, driver_inputs)                     # sync vis with driver

        driver.Advance(step_size)                                    # advance driver
        terrain.Advance(step_size)                                   # advance terrain
        artcar.Advance(step_size)                                    # advance vehicle (steps system)
        vis.Advance(step_size)                                       # advance vis
        manager.Update()                                             # update all sensors

        if artcar.GetSystem().GetChTime() >= sim_end:
            break

    realtime_timer.Spin(step_size)                                   # real-time pacing spin
