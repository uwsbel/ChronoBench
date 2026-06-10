import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                  # bundled data root
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')              # vehicle data root

# --- ARTcar vehicle (its system becomes the main physical system) ---
initLoc = chrono.ChVector3d(0, 0, 0.5)                                # initial chassis location
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                            # initial chassis rotation (QUNIT)

vehicle = veh.ARTcar()                                                # self-owning catalog wrapper
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                  # rigid-terrain contact
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)              # no chassis collision shape
vehicle.SetChassisFixed(False)                                        # chassis free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))        # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                        # TMEASY tires
vehicle.SetTireStepSize(1e-3)                                         # tire integration step
vehicle.SetMaxMotorVoltageRatio(0.16)                                # ARTcar motor voltage ratio
vehicle.SetStallTorque(0.3)                                          # ARTcar stall torque
vehicle.SetTireRollingResistance(0.06)                              # ARTcar rolling resistance
vehicle.Initialize()                                                 # build the vehicle + its system

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)      # chassis mesh
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)   # suspension mesh
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)     # steering mesh
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)        # wheel mesh
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)         # tire mesh

sys = vehicle.GetSystem()                                            # the vehicle-owned system is the main system
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)     # collision system (terrain contact)

print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())              # truth diagnostic banner

# --- rigid terrain ---
terrain = veh.RigidTerrain(sys)                                      # rigid terrain on the vehicle system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC terrain material
patch_mat.SetFriction(0.9)                                            # friction coefficient
patch_mat.SetRestitution(0.01)                                        # restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)   # 100x100 flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)  # terrain texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                        # terrain color
terrain.Initialize()                                                 # finalize terrain

# --- driver (default settings) ---
driver = veh.ChDriver(vehicle.GetVehicle())                          # default driver
driver.Initialize()

# --- sensor manager on the vehicle system ---
manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)   # scene light for the camera
manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)
manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(1, 1, 1), 500.0)

chassis_body = vehicle.GetChassisBody()                              # body all sensors ride on

# --- lidar parameters shared by both lidars ---
update_rate = 5.0                                                   # lidar update rate (Hz)
horizontal_samples = 800                                            # horizontal beams
vertical_samples = 300                                              # vertical beams (3D lidar)
horizontal_fov = 2 * chrono.CH_PI                                   # 360 deg horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                  # upper vertical FOV
min_vert_angle = -chrono.CH_PI / 6                                  # lower vertical FOV
max_lidar_range = 100.0                                             # max range (m)
sample_radius = 2                                                   # beam super-sample radius
divergence_angle = 0.003                                           # beam divergence (rad)

lidar_offset_pose = chrono.ChFramed(                                # lidar mount pose on the chassis
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QUNIT,
)

# --- 3D lidar sensor attached to the chassis ---
lidar = sens.ChLidarSensor(
    chassis_body,                                                  # body the lidar rides on
    update_rate,                                                   # update rate (Hz)
    lidar_offset_pose,                                             # offset pose on the chassis
    horizontal_samples,                                            # horizontal samples
    vertical_samples,                                              # vertical samples
    horizontal_fov,                                                # horizontal FOV (rad)
    max_vert_angle,                                                # max vertical angle (rad)
    min_vert_angle,                                                # min vertical angle (rad)
    max_lidar_range,                                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                               # beam shape
    sample_radius,                                                 # sample radius
    divergence_angle,                                              # vertical divergence
    divergence_angle,                                              # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                         # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(0)                                                    # lag = 0
lidar.SetCollectionWindow(1.0 / update_rate)                       # collection window = 1 / rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                          # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                       # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterSavePtCloud("cam/lidar/"))           # save the 3D lidar point cloud
lidar.PushFilter(sens.ChFilterXYZIAccess())                        # host access to XYZI
manager.AddSensor(lidar)                                           # register the 3D lidar

# --- 2D lidar sensor (single vertical channel) attached to the chassis ---
lidar_2d = sens.ChLidarSensor(
    chassis_body,                                                  # body the 2D lidar rides on
    update_rate,                                                   # update rate (Hz)
    lidar_offset_pose,                                             # offset pose on the chassis
    horizontal_samples,                                            # horizontal samples
    1,                                                             # vertical samples = 1 (2D scan)
    horizontal_fov,                                                # horizontal FOV (rad)
    0.0,                                                           # max vertical angle = 0 (2D)
    0.0,                                                           # min vertical angle = 0 (2D)
    max_lidar_range,                                               # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                               # beam shape
    sample_radius,                                                 # sample radius
    divergence_angle,                                              # vertical divergence
    divergence_angle,                                              # horizontal divergence
    sens.LidarReturnMode_STRONGEST_RETURN,                         # return mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(0)                                                 # lag = 0
lidar_2d.SetCollectionWindow(1.0 / update_rate)                    # collection window = 1 / rate
lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))  # depth preview
lidar_2d.PushFilter(sens.ChFilterDIAccess())                       # host access to depth+intensity
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())                    # depth -> XYZI point cloud
lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "2D Lidar Point Cloud"))  # point-cloud preview
lidar_2d.PushFilter(sens.ChFilterSavePtCloud("cam/lidar_2d/"))     # save the 2D lidar point cloud
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())                     # host access to XYZI
manager.AddSensor(lidar_2d)                                        # register the 2D lidar

# --- third-person camera attached to the chassis ---
cam_offset_pose = chrono.ChFramed(                                 # third-person view behind/above the chassis
    chrono.ChVector3d(-8, 0, 1.45),
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)),
)
cam = sens.ChCameraSensor(
    chassis_body,                                                 # body the camera rides on
    30,                                                           # update rate (Hz)
    cam_offset_pose,                                              # offset pose on the chassis
    1280,                                                         # image width
    720,                                                          # image height
    1.408,                                                        # horizontal FOV (rad)
)
cam.SetName("Third Person POV")
cam.SetLag(0)                                                     # lag = 0
cam.SetCollectionWindow(0)                                        # exposure / collection window = 0
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Third Person Camera"))  # live preview
cam.PushFilter(sens.ChFilterRGBA8Access())                       # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                    # save color PNGs
manager.AddSensor(cam)                                            # register the camera

# --- simulation loop (headless, time-bounded, vehicle Synchronize/Advance) ---
step_size = 1e-3                                                   # physics step
time_end = 40.0                                                    # scored end time
time = 0.0                                                        # simulation clock


while time < time_end:
    time = sys.GetChTime()                                       # current sim time

    driver_inputs = driver.GetInputs()                           # current driver inputs

    driver.Synchronize(time)                                     # synchronize driver
    terrain.Synchronize(time)                                    # synchronize terrain
    vehicle.Synchronize(time, driver_inputs, terrain)            # synchronize vehicle
    manager.Update()                                             # pump all sensors once per step

    driver.Advance(step_size)                                    # advance driver
    terrain.Advance(step_size)                                   # advance terrain
    vehicle.Advance(step_size)                                   # advance vehicle (steps the system)

    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()                # read the 3D lidar point cloud
    if xyzi_buffer.HasData():                                    # only after the first lidar tick
        xyzi = xyzi_buffer.GetXYZIData()
        print("XYZI buffer received. Resolution: {0}x{1}".format(xyzi_buffer.Width, xyzi_buffer.Height))
        if xyzi.size:                                            # guard empty scans
            print("Max XYZI value: ", np.max(xyzi))

    xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()          # read the 2D lidar point cloud
    if xyzi_buffer_2d.HasData():                                 # only after the first 2D lidar tick
        xyzi_2d = xyzi_buffer_2d.GetXYZIData()
        print("2D XYZI buffer received. Resolution: {0}x{1}".format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height))
        if xyzi_2d.size:                                         # guard empty single-row scans
            print("Max 2D XYZI value: ", np.max(xyzi_2d))

    rgba_buffer = cam.GetMostRecentRGBA8Buffer()                 # read the third-person camera frame
    if rgba_buffer.HasData():                                    # only after the first camera tick
        print("Camera buffer received. Resolution: {0}x{1}".format(rgba_buffer.Width, rgba_buffer.Height))
