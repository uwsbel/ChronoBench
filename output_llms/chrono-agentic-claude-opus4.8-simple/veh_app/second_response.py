import os
import math
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                 # core data path
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')             # vehicle data path

initLoc = chrono.ChVector3d(0, -5, 0.4)                              # Gator spawn
initRot = chrono.ChQuaterniond(1, 0, 0, 0)                           # QUNIT (identity)

gator = veh.Gator()                                                  # self-owning Gator wrapper
gator.SetContactMethod(chrono.ChContactMethod_NSC)                   # rigid terrain -> NSC
gator.SetChassisCollisionType(veh.CollisionType_NONE)               # no chassis collision shape
gator.SetChassisFixed(False)                                         # chassis free to move
gator.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))          # initial pose
gator.SetBrakeType(veh.BrakeType_SHAFTS)                             # shafts-based brake
gator.SetTireType(veh.TireModelType_TMEASY)                          # TMeasy tires
gator.SetTireStepSize(1e-3)                                          # tire integration step
gator.SetInitFwdVel(0.0)                                             # start from rest
gator.Initialize()                                                   # build the vehicle

system = gator.GetSystem()                                           # wrapper-owned system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # collision system (contact scene)

print("VEHICLE MASS: ", gator.GetVehicle().GetMass())                                 # mass banner
print("DRIVELINE TEMPLATE: ", gator.GetVehicle().GetDriveline().GetTemplateName())    # driveline template
print("TIRE TEMPLATE: ", gator.GetVehicle().GetTire(0, veh.LEFT).GetTemplateName())   # tire template

gator.SetChassisVisualizationType(veh.VisualizationType_MESH)        # chassis: mesh
gator.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)  # suspension: primitives
gator.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)    # steering: primitives
gator.SetWheelVisualizationType(veh.VisualizationType_NONE)          # wheels: none
gator.SetTireVisualizationType(veh.VisualizationType_MESH)           # tires: mesh

terrain = veh.RigidTerrain(system)                                   # rigid terrain owned by system
patch_mat = chrono.ChContactMaterialNSC()                            # NSC contact material
patch_mat.SetFriction(0.9)                                           # terrain friction
patch_mat.SetRestitution(0.01)                                       # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 50.0, 50.0)     # 50x50 patch at origin
patch.SetColor(chrono.ChColor(0.8, 0.8, 1.0))                        # patch color
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 50, 50)  # tiled texture
terrain.Initialize()                                                 # build terrain

obstacle_mat = chrono.ChContactMaterialNSC()                         # contact material for obstacles
box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, obstacle_mat)  # 1x1x1 box (collision on)
box.SetPos(chrono.ChVector3d(0, 0, 0.5))                             # box position
box.SetFixed(True)                                                   # static obstacle
box.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/blue.png"))  # blue texture
system.Add(box)                                                      # add box to system

cyl = chrono.ChBodyEasyCylinder(chrono.ChAxis_Z, 0.5, 1, 1000, True, True, obstacle_mat)  # r=0.5 h=1 cylinder
cyl.SetPos(chrono.ChVector3d(0, 0, 1.5))                             # cylinder position
cyl.SetFixed(True)                                                   # static obstacle
cyl.GetVisualShape(0).SetTexture(veh.GetDataFile("terrain/textures/blue.png"))  # blue texture
system.Add(cyl)                                                      # add cylinder to system

driver = veh.ChDriver(gator.GetVehicle())                            # plain (non-interactive) driver
driver.Initialize()                                                  # initialize driver
driver.SetSteering(0.5)                                              # scripted steering
driver.SetThrottle(0.2)                                              # scripted throttle

manager = sens.ChSensorManager(gator.GetSystem())                    # sensor manager over the system
manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100),
                            chrono.ChColor(1, 1, 1), 500.0)          # scene point light

cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 1.45),                                  # third-person offset on chassis
    chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(0, 1, 0)))       # slight downward tilt
cam = sens.ChCameraSensor(
    gator.GetChassisBody(),                                          # mounted on the chassis body
    10,                                                              # update_rate (Hz)
    cam_offset_pose,                                                 # offset pose on the chassis
    1280, 720,                                                       # image width, height
    1.408)                                                           # horizontal FOV (rad)
cam.SetName("Third Person POV")                                      # sensor name
cam.SetLag(0)                                                        # no lag
cam.SetCollectionWindow(0)                                           # zero exposure window
cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Gator Camera"))    # live RGB preview
cam.PushFilter(sens.ChFilterRGBA8Access())                           # host access to RGBA8 buffer
cam.PushFilter(sens.ChFilterSave("cam/rgb/"))                        # save RGB frames
manager.AddSensor(cam)                                               # register the camera

lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, 0, 2),                                    # lidar offset on chassis
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))         # no tilt
lidar = sens.ChLidarSensor(
    gator.GetChassisBody(),                                          # mounted on the chassis body
    5.0,                                                             # update_rate (Hz)
    lidar_offset_pose,                                               # offset pose
    800,                                                             # horizontal samples
    300,                                                             # vertical channels
    2 * chrono.CH_PI,                                                # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                               # max vertical angle
    -chrono.CH_PI / 6,                                               # min vertical angle
    100.0,                                                           # max range
    sens.LidarBeamShape_RECTANGULAR,                                 # rectangular beam shape
    2,                                                               # sample radius
    0.003,                                                           # vertical divergence angle
    0.003,                                                           # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN)                           # strongest return mode
lidar.SetName("Lidar Sensor")                                        # sensor name
lidar.SetLag(0)                                                      # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                                 # collection window = 1/update_rate
lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw Lidar Depth"))  # raw depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                            # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                         # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                          # host access to XYZI
manager.AddSensor(lidar)                                             # register the lidar

step_size = 1e-3                                                     # physics step
end_time = 30.0                                                      # simulation duration

time = 0
while time < end_time:                                               # headless time-bounded loop
    time = system.GetChTime()                                        # simulation clock

    driver.SetSteering(0.5)                                          # steering each step
    driver.SetThrottle(0.2)                                          # throttle each step
    driver_inputs = driver.GetInputs()                              # current driver inputs

    driver.Synchronize(time)                                         # advance driver state
    terrain.Synchronize(time)                                        # advance terrain
    gator.Synchronize(time, driver_inputs, terrain)                 # feed inputs to vehicle

    driver.Advance(step_size)                                        # step driver
    terrain.Advance(step_size)                                       # step terrain
    gator.Advance(step_size)                                         # step vehicle (steps the system)

    manager.Update()                                                 # pump sensors once per step

    rgba = cam.GetMostRecentRGBA8Buffer()                            # read camera buffer
    if rgba.HasData():                                               # only after first camera tick
        print('Camera buffer received. Resolution: {0}x{1}'.format(rgba.Width, rgba.Height))
    xyzi = lidar.GetMostRecentXYZIBuffer()                           # read lidar buffer
    if xyzi.HasData():                                               # only after first lidar tick
        print('Lidar buffer received. Resolution: {0}x{1}'.format(xyzi.Width, xyzi.Height))
