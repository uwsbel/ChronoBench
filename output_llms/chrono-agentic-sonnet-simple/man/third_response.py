import os
import pychrono.core as chrono
import pychrono.irrlicht as irr
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import math

chrono.SetChronoDataPath(chrono.GetChronoDataPath())                     # locate bundled Chrono assets
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')                 # locate vehicle data files

# Initial vehicle location and orientation
initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

vis_type = veh.VisualizationType_MESH                                    # mesh visuals for all parts
chassis_collision_type = veh.CollisionType_NONE                          # no chassis collision mesh
tire_model = veh.TireModelType_TMEASY                                    # TMEASY tire model

terrainHeight = 0                                                        # terrain height
terrainLength = 100.0                                                    # terrain size in X
terrainWidth = 100.0                                                     # terrain size in Y

trackPoint = chrono.ChVector3d(-3.0, 0.0, 1.1)                          # chassis point tracked by camera

contact_method = chrono.ChContactMethod_NSC                              # NSC contact for rigid terrain

step_size = 1e-3                                                         # simulation timestep
tire_step_size = step_size                                               # tire substep same as sim step

render_step_size = 1.0 / 50                                              # render at 50 FPS
render_fps = 50.0                                                        # FPS for review recording

# Create the MAN 10t vehicle
vehicle = veh.MAN_10t()
vehicle.SetContactMethod(contact_method)
vehicle.SetChassisCollisionType(chassis_collision_type)
vehicle.SetChassisFixed(False)                                           # chassis must be free to move
vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
vehicle.SetTireType(tire_model)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

vehicle.SetChassisVisualizationType(vis_type)
vehicle.SetSuspensionVisualizationType(vis_type)
vehicle.SetSteeringVisualizationType(vis_type)
vehicle.SetWheelVisualizationType(vis_type)
vehicle.SetTireVisualizationType(vis_type)

vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)  # REQUIRED for contact

# Randomly create 5 boxes and add them to the scene
for i in range(5):
    side = np.random.uniform(0.5, 1.5)                                   # random side length
    box = chrono.ChBodyEasyBox(side, side, side, 1000, True, False)
    box.SetPos(chrono.ChVector3d(np.random.uniform(-10, 10), np.random.uniform(-10, 10), side / 2))
    box.SetFixed(True)
    box.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    vehicle.GetSystem().Add(box)

# Create the rigid terrain with grass texture
patch_mat = chrono.ChContactMaterialNSC()                                # NSC material matches contact method
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch = terrain.AddPatch(
    patch_mat,
    chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
    terrainLength, terrainWidth,
)
patch.SetTexture(veh.GetDataFile("terrain/textures/grass.jpg"), 200, 200)  # grass texture
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
terrain.Initialize()

# Create the vehicle Irrlicht visualization
vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
vis.SetWindowTitle('MAN 10t Demo')
vis.SetWindowSize(1280, 1024)
vis.SetChaseCamera(trackPoint, 15.0, 0.5)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddLightDirectional()                                                # vehicle demos use directional light
vis.AddSkyBox()
vis.AttachVehicle(vehicle.GetVehicle())

# Create sensor manager and lidar sensor
manager = sens.ChSensorManager(vehicle.GetSystem())                      # sensor manager on vehicle system

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(2.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)),
)                                                                        # lidar offset on chassis

# Lidar parameters
update_rate = 5.0                                                        # scanning rate in Hz
horizontal_samples = 800                                                 # horizontal sample count
vertical_samples = 300                                                   # vertical channel count
horizontal_fov = 2 * chrono.CH_PI                                       # 360-degree horizontal FOV
max_vert_angle = chrono.CH_PI / 12                                       # max vertical angle
min_vert_angle = -chrono.CH_PI / 6                                       # min vertical angle
lag = 0                                                                  # lidar lag time
collection_time = 1.0 / update_rate                                      # collection window = 1/rate
sample_radius = 2                                                        # samples per ray
divergence_angle = 0.003                                                 # beam divergence (3 mm)
return_mode = sens.LidarReturnMode_STRONGEST_RETURN                      # strongest return mode

lidar = sens.ChLidarSensor(
    vehicle.GetChassis().GetBody(),   # attached to chassis body
    update_rate,                       # scanning rate Hz
    offset_pose,                       # offset pose on chassis
    horizontal_samples,                # horizontal samples
    vertical_samples,                  # vertical channels
    horizontal_fov,                    # horizontal FOV
    max_vert_angle,                    # max vertical angle
    min_vert_angle,                    # min vertical angle
    100.0,                             # max range
    sens.LidarBeamShape_RECTANGULAR,   # beam shape
    sample_radius,                     # sample radius
    divergence_angle,                  # vertical divergence
    divergence_angle,                  # horizontal divergence
    return_mode,                       # return mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)
lidar.PushFilter(sens.ChFilterDIAccess())                                # host access to depth+intensity
lidar.PushFilter(sens.ChFilterPCfromDepth())                             # depth -> XYZI point cloud
lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))  # live preview
lidar.PushFilter(sens.ChFilterXYZIAccess())                              # host access to XYZI
manager.AddSensor(lidar)                                                 # register lidar with manager

# Create interactive driver
driver = veh.ChInteractiveDriverIRR(vis)

steering_time = 1.0                                                      # seconds to go 0->+1 steering
throttle_time = 1.0                                                      # seconds to go 0->+1 throttle
braking_time = 0.3                                                       # seconds to go 0->+1 braking
driver.SetSteeringDelta(render_step_size / steering_time)
driver.SetThrottleDelta(render_step_size / throttle_time)
driver.SetBrakingDelta(render_step_size / braking_time)
driver.Initialize()

# Output vehicle mass
print("VEHICLE MASS: ", vehicle.GetVehicle().GetMass())

# Simulation loop setup
render_steps = math.ceil(render_step_size / step_size)                   # physics steps per render frame
render_every = max(1, round(1.0 / (render_fps * step_size)))             # cadence constant

realtime_timer = chrono.ChRealtimeStepTimer()                            # real-time pacing timer
step_number = 0
render_frame = 0


while vis.Run():
    time = vehicle.GetSystem().GetChTime()

    if (step_number % render_steps == 0):
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame += 1

    driver_inputs = driver.GetInputs()

    # Update sensor manager
    manager.Update()

    # Synchronize all modules
    driver.Synchronize(time)
    terrain.Synchronize(time)
    vehicle.Synchronize(time, driver_inputs, terrain)
    vis.Synchronize(time, driver_inputs)

    # Advance all modules
    driver.Advance(step_size)
    terrain.Advance(step_size)
    vehicle.Advance(step_size)
    vis.Advance(step_size)


    step_number += 1
    realtime_timer.Spin(step_size)                                       # real-time pacing
