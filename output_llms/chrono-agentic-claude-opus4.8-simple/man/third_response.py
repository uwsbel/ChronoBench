import os
import math
import random
import numpy as np
import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                   # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')               # locate vehicle data files

init_loc = chrono.ChVector3d(0, 0, 0.5)                                # initial chassis location
init_rot = chrono.ChQuaterniond(1, 0, 0, 0)                            # QUNIT, no initial rotation

step_size = 1e-3                                                        # integration step
tire_step_size = step_size                                             # tire force model step
render_step_size = 1.0 / 50.0                                          # 50 FPS render cadence

vehicle = veh.MAN_10t()                                                # MAN 10t catalog truck
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)                   # NSC for rigid terrain
vehicle.SetChassisCollisionType(veh.CollisionType_NONE)                # no chassis collision shape
vehicle.SetChassisFixed(False)                                         # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))        # spawn pose
vehicle.SetTireType(veh.TireModelType_TMEASY)                          # TMEASY tire model
vehicle.SetTireStepSize(tire_step_size)                                # tire integration step
vehicle.Initialize()                                                   # build the vehicle subsystems

system = vehicle.GetSystem()                                           # the wrapper-owned ChSystem
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)    # REQUIRED for contact
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())                # report total vehicle mass

vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)        # mesh chassis
vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)     # mesh suspension
vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)       # mesh steering
vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)          # mesh wheels
vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)           # mesh tires

terrain = veh.RigidTerrain(system)                                     # rigid (flat) terrain
patch_mat = chrono.ChContactMaterialNSC()                              # NSC contact material
patch_mat.SetFriction(0.9)                                             # terrain friction
patch_mat.SetRestitution(0.01)                                         # terrain restitution
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 100.0, 100.0)     # 100x100 m flat patch
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))                          # patch color
terrain.Initialize()                                                   # build the terrain

box_mat = chrono.ChContactMaterialNSC()                                # contact material for the boxes
random.seed(0)                                                         # reproducible random layout
for i in range(10):                                                    # scatter random boxes around the truck
    size = random.uniform(0.5, 1.5)                                    # random cube edge length
    bx = random.uniform(-20.0, 20.0)                                   # random X position
    by = random.uniform(-20.0, 20.0)                                   # random Y position
    box = chrono.ChBodyEasyBox(size, size, size, 1000.0, True, True, box_mat)  # visual + collision box
    box.SetPos(chrono.ChVector3d(bx, by, size / 2.0))                  # rest on the ground
    box.SetFixed(True)                                                 # static obstacle
    box.GetVisualShape(0).SetColor(chrono.ChColor(0.4, 0.5, 0.6))      # box color
    system.Add(box)                                                    # add the box to the world

manager = sens.ChSensorManager(system)                                 # oversees all sensors

offset_pose = chrono.ChFramed(                                         # lidar mounting pose on the chassis
    chrono.ChVector3d(-2.0, 0, 2.5),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)
horizontal_samples = 800                                               # lidar horizontal beam count
vertical_samples = 300                                                 # lidar vertical beam count
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),                                          # mount on the truck chassis
    5.0,                                                               # update_rate (Hz)
    offset_pose,                                                       # offset pose
    horizontal_samples,                                                # horizontal samples
    vertical_samples,                                                  # vertical samples
    2 * chrono.CH_PI,                                                  # 360 deg horizontal FOV
    chrono.CH_PI / 12,                                                 # max vertical angle
    -chrono.CH_PI / 6,                                                 # min vertical angle
    100.0,                                                             # max range (m)
    sens.LidarBeamShape_RECTANGULAR,                                   # beam shape
    2,                                                                 # sample radius
    0.003,                                                             # vertical divergence angle
    0.003,                                                             # horizontal divergence angle
    sens.LidarReturnMode_STRONGEST_RETURN,                             # strongest return
)
lidar.SetName("Lidar Sensor")                                          # sensor name
lidar.SetLag(0)                                                        # no lag
lidar.SetCollectionWindow(1.0 / 5.0)                                   # collection window = 1/update_rate
lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth"))  # depth preview
lidar.PushFilter(sens.ChFilterDIAccess())                              # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                           # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # point-cloud preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                            # host access to XYZI cloud
manager.AddSensor(lidar)                                               # register the lidar

vis = veh.ChWheeledVehicleVisualSystemIrrlicht()                       # vehicle Irrlicht visual system
vis.SetWindowTitle('MAN 10t Demo')                                     # window title
vis.SetWindowSize(1280, 1024)                                          # window size
vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 15.0, 0.5)       # chase camera trackpoint/dist/height
vis.Initialize()                                                       # create the Irrlicht device FIRST
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))       # logo (after Initialize)
vis.AddLightDirectional()                                              # directional light (vehicle style)
vis.AddSkyBox()                                                        # sky box
vis.AttachVehicle(vehicle.GetVehicle())                                # bind the vehicle to the chase cam

driver = veh.ChInteractiveDriverIRR(vis)                               # real-time interactive driver
driver.SetSteeringDelta(render_step_size / 1.0)                        # steering rate
driver.SetThrottleDelta(render_step_size / 1.0)                        # throttle rate
driver.SetBrakingDelta(render_step_size / 0.3)                         # braking rate
driver.Initialize()                                                    # build the driver

render_steps = math.ceil(render_step_size / step_size)                 # physics steps per render frame
realtime_timer = chrono.ChRealtimeStepTimer()                          # wall-clock pacing
step_number = 0                                                        # step counter


while vis.Run():                                                       # plain real-time loop (truth form)
    time = vehicle.GetSystem().GetChTime()                            # current sim time

    if step_number % render_steps == 0:                               # throttled rendering
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    driver_inputs = driver.GetInputs()                                # current driver command

    driver.Synchronize(time)                                          # advance driver state
    terrain.Synchronize(time)                                         # advance terrain
    vehicle.Synchronize(time, driver_inputs, terrain)                 # feed inputs to the vehicle
    vis.Synchronize(time, driver_inputs)                              # sync the visual system

    driver.Advance(step_size)                                         # step driver
    terrain.Advance(step_size)                                        # step terrain
    vehicle.Advance(step_size)                                        # step the wrapper-owned system
    vis.Advance(step_size)                                            # step the visual system

    manager.Update()                                                  # update all sensors once per step

    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()                     # read the latest point cloud
    if xyzi_buffer.HasData():                                         # report on new lidar data
        print('Lidar buffer. Points: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))

    step_number += 1                                                  # advance counter
    realtime_timer.Spin(step_size)                                    # match wall-clock to sim time
