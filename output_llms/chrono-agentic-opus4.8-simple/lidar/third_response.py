import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # locate vehicle data files

step_size = 1e-3                                                       # physics step
tire_step_size = 1e-3                                                  # tire integration step

init_loc = chrono.ChVector3d(0, 0, 0.5)                               # vehicle spawn (z = suspension ref)
init_rot = chrono.QuatFromAngleZ(0)                                   # facing +X

car = veh.ARTcar()                                                    # ARTcar wheeled vehicle wrapper
car.SetContactMethod(chrono.ChContactMethod_NSC)                      # NSC for rigid terrain
car.SetChassisCollisionType(veh.CollisionType_NONE)                  # no chassis collision mesh
car.SetChassisFixed(False)                                            # MANDATORY — fixed chassis won't move
car.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))          # initial pose
car.SetTireType(veh.TireModelType_TMEASY)                            # TMEASY tires on rigid terrain
car.SetTireStepSize(tire_step_size)                                  # tire substep
car.Initialize()                                                     # build the vehicle

car.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)    # chassis viz
car.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension viz
car.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)   # steering viz
car.SetWheelVisualizationType(veh.VisualizationType_MESH)            # wheel viz
car.SetTireVisualizationType(veh.VisualizationType_MESH)             # tire viz

system = car.GetSystem()                                              # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact
print("VEHICLE MASS: ", car.GetVehicle().GetMass())                  # report total vehicle mass

terrain = veh.RigidTerrain(system)                                   # rigid flat terrain
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)  # 100 x 100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                       # terrain color
terrain.Initialize()                                                 # build terrain

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                     # vehicle Irrlicht window
vis.SetWindowTitle("ARTcar Lidar")                                   # window title
vis.SetWindowSize(1280, 720)                                         # window size
vis.SetChaseCamera(chrono.ChVector3d(0, 0, 0.5), 6.0, 0.5)         # chase camera on chassis
vis.Initialize()                                                     # build the device first
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))    # logo
vis.AddSkyBox()                                                     # sky box
vis.AddLightDirectional()                                          # directional light (vehicle truth)
vis.AttachVehicle(car.GetVehicle())                                 # bind vehicle visual assets

driver = veh.ChInteractiveDriverIRR(vis)                            # interactive driver bound to vis
steering_time = 1.0                                                 # s to full steering
throttle_time = 1.0                                                 # s to full throttle
braking_time = 0.3                                                  # s to full brake
render_step_size = 1.0 / 50.0                                       # render cadence (s)
driver.SetSteeringDelta(render_step_size / steering_time)          # steering rate
driver.SetThrottleDelta(render_step_size / throttle_time)          # throttle rate
driver.SetBrakingDelta(render_step_size / braking_time)            # braking rate
driver.Initialize()                                                # build the driver

chassis_body = car.GetChassisBody()                                # chassis body for sensors

manager = sens.ChSensorManager(system)                            # sensor manager
intensity = 1.0                                                   # light intensity
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)  # point light
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100),
                            chrono.ChColor(intensity, intensity, intensity), 500.0)  # point light

lidar_offset = chrono.ChFramed(chrono.ChVector3d(1.0, 0, 1),
                               chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))  # lidar offset on chassis

lidar = sens.ChLidarSensor(                                        # 3D lidar on the chassis
    chassis_body,
    5.0,                                                          # update_rate (Hz)
    lidar_offset,                                                 # offset pose
    800,                                                         # horizontal samples
    300,                                                         # vertical samples (3D)
    2 * chrono.CH_PI,                                            # horizontal FOV
    chrono.CH_PI / 12,                                           # max vertical angle
    -chrono.CH_PI / 6,                                           # min vertical angle
    100.0,                                                      # max range
    sens.LidarBeamShape_RECTANGULAR,                            # beam shape
    2,                                                          # sample radius
    0.003,                                                     # vert divergence
    0.003,                                                     # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                     # return mode
)
lidar.SetName("Lidar Sensor 3D")                                # name
lidar.SetLag(0)                                                 # lag = 0
lidar.SetCollectionWindow(1.0 / 5.0)                          # collection window = 1/update_rate
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))  # raw depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                     # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                 # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                  # host access to XYZI
manager.AddSensor(lidar)                                      # register 3D lidar

lidar_2d = sens.ChLidarSensor(                                # 2D lidar on the chassis
    chassis_body,
    5.0,                                                     # update_rate (Hz)
    lidar_offset,                                            # offset pose
    800,                                                    # horizontal samples
    1,                                                      # vertical samples (2D)
    2 * chrono.CH_PI,                                       # horizontal FOV
    0,                                                     # max vertical angle (2D)
    0,                                                     # min vertical angle (2D)
    100.0,                                                # max range
    sens.LidarBeamShape_RECTANGULAR,                      # beam shape
    2,                                                    # sample radius
    0.003,                                               # vert divergence
    0.003,                                               # hori divergence
    sens.LidarReturnMode_STRONGEST_RETURN,               # return mode
)
lidar_2d.SetName("Lidar Sensor 2D")                       # name
lidar_2d.SetLag(0)                                       # lag = 0
lidar_2d.SetCollectionWindow(1.0 / 5.0)                # collection window
lidar_2d.PushFilter(sens.ChFilterVisualize(800, 100, "Raw Lidar 2D Depth"))  # raw 2D depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())            # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())        # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar 2D Point Cloud"))  # point cloud preview
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())         # host access to XYZI
manager.AddSensor(lidar_2d)                            # register 2D lidar

cam_offset = chrono.ChFramed(chrono.ChVector3d(-6.0, 0, 2.0),
                             chrono.QuatFromAngleAxis(0.1, chrono.ChVector3d(0, 1, 0)))  # third-person offset

cam = sens.ChCameraSensor(                              # third-person camera on the chassis
    chassis_body,
    30,                                                # update_rate (Hz)
    cam_offset,                                        # offset pose
    1280, 720,                                         # width, height
    1.408,                                             # horizontal FOV
)
cam.SetName("Third Person Camera")                     # name
cam.SetLag(0)                                          # lag = 0
cam.SetCollectionWindow(0)                             # collection window = 0
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person View"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())             # host access to RGBA8
cam.PushFilter(sens.ChFilterSave("cam/third_person/"))  # save color PNGs
manager.AddSensor(cam)                                 # register camera

render_steps = math.ceil(render_step_size / step_size)  # untagged render cadence
realtime_timer = chrono.ChRealtimeStepTimer()         # real-time spin timer
step_number = 0                                       # loop step counter
while vis.Run():
    time = system.GetChTime()                         # current sim time

    if step_number % render_steps == 0:               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                # current driver inputs

    driver.Synchronize(time)                          # sync driver
    terrain.Synchronize(time)                         # sync terrain
    car.Synchronize(time, driver_inputs, terrain)     # sync vehicle
    vis.Synchronize(time, driver_inputs)              # sync vis

    driver.Advance(step_size)                         # advance driver
    terrain.Advance(step_size)                        # advance terrain
    car.Advance(step_size)                            # advance vehicle (steps the system)
    vis.Advance(step_size)                            # advance vis

    manager.Update()                                  # pump sensors once per step

    di_buf = lidar.GetMostRecentDIBuffer()            # 3D lidar depth+intensity buffer
    if di_buf.HasData():                              # only read after first tick
        print('Lidar3D buffer received. Beams: {0}x{1}'.format(di_buf.Width, di_buf.Height))

    di2_buf = lidar_2d.GetMostRecentDIBuffer()        # 2D lidar depth+intensity buffer
    if di2_buf.HasData():                             # only read after first tick
        print('Lidar2D buffer received. Beams: {0}x{1}'.format(di2_buf.Width, di2_buf.Height))

    step_number += 1                                  # advance loop counter
    realtime_timer.Spin(step_size)                    # spin to match wall-clock
