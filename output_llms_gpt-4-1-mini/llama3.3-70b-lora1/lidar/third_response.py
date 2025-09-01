import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import numpy as np
import math
import time as time_module

# =============================================================================

# Create systems

# vehicle system
vehicle = veh.ARTcar()
vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
vehicle.SetChassisFixed(False)
vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 1), chrono.ChQuaterniond(1, 0, 0, 0)))
vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
vehicle.SetDriveType(veh.DrivelineTypeWV_RWD)
vehicle.SetSteeringType(veh.SteeringTypeWV_PITMAN_ARM)
vehicle.SetTireType(veh.TireModelType_TMEASY)
vehicle.SetTireStepSize(tire_step_size)
vehicle.Initialize()

# driver system
driver = veh.ChDriver(vehicle.GetVehicle())
driver.Initialize()

# terrain system
terrain = veh.RigidTerrain(vehicle.GetSystem())
patch_mat = chrono.ChContactMaterialNSC()
patch_mat.SetFriction(0.9)
patch_mat.SetRestitution(0.01)
patch = terrain.AddPatch(patch_mat, chrono.CSYSNORM, 600, 600)
patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 600, 600)
terrain.Initialize()

# Create the sensor manager and add a lidar and camera
manager = sens.ChSensorManager(vehicle.GetVehicle())
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)

lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),
    update_rate,            # Scanning rate in Hz
    offset_pose,            # Offset pose
    horizontal_samples,     # Number of horizontal samples
    vertical_samples,       # Number of vertical channels
    horizontal_fov,         # Horizontal field of view
    max_vert_angle,         # Maximum vertical field of view
    min_vert_angle,         # Minimum vertical field of view
    100.0,                  # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
    sample_radius,          # Sample radius
    divergence_angle,       # Divergence angle
    divergence_angle,       # Divergence angle (again, typically same value)
    return_mode             # Return mode for the lidar
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

if noise_model == "CONST_NORMAL_XYZI":
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
elif noise_model == "NONE":
    # Don't add any noise models
    pass
if vis:
    # Visualize the raw lidar data
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
# Provides the host access to the Depth, Intensity data
lidar.PushFilter(sens.ChFilterDIAccess())
# Convert Depth, Intensity data to XYZI point cloud data
lidar.PushFilter(sens.ChFilterPCfromDepth())
if vis:
    # Visualize the point cloud
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
# Provides the host access to the XYZI data
lidar.PushFilter(sens.ChFilterXYZIAccess())
# Add the lidar to the sensor manager
manager.AddSensor(lidar)

# Create 2D lidar sensor
lidar_2d =  sens.ChLidarSensor(
    vehicle.GetChassisBody(),              # Body lidar is attached to
    update_rate,            # Scanning rate in Hz
    offset_pose,            # Offset pose
    horizontal_samples,     # Number of horizontal samples
    1,                      # only 1 vertical channel for 2D lidar
    horizontal_fov,         # Horizontal field of view
    0.0,                    # Maximum vertical field of view
    0.0,                    # Minimum vertical field of view
    100.0,                  # Maximum lidar range
    sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
    sample_radius,          # Sample radius
    divergence_angle,       # Divergence angle
    divergence_angle,       # Divergence angle (again, typically same value)
    return_mode             # Return mode for the lidar
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(lag)
lidar_2d.SetCollectionWindow(collection_time)
if noise_model == "CONST_NORMAL_XYZI":
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
elif noise_model == "NONE":
    # Don't add any noise models
    pass
if vis:
    # Visualize the raw lidar data
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
# Provides the host access to the Depth, Intensity data
lidar_2d.PushFilter(sens.ChFilterDIAccess())
# Convert Depth, Intensity data to XYZI point cloud data
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
# Provides the host access to the XYZI data
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
# Add the lidar to the sensor manager
manager.AddSensor(lidar_2d)

offset_pose = chrono.ChFramed(
    chrono.ChVector3d(-8, 0, 2),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
cam = sens.ChCameraSensor(
    vehicle.GetChassisBody(),              # Body the camera is attached to
    update_rate,            # Update rate in Hz
    offset_pose,            # Offset pose
    image_width,            # Image width
    image_height,           # Image height
    fov                     # Camera's horizontal field of view
)
cam.SetName("Camera Sensor")
cam.SetLag(lag)
cam.SetCollectionWindow(collection_time)
if vis:
    # Create a filter graph with a visualization step
    cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera Image"))
# Provides the host access to the RGB data
cam.PushFilter(sens.ChFilterRGBAAccess())
# Add the camera to the sensor manager
manager.AddSensor(cam)

# ---------------
# Simulation loop
# ---------------
orbit_radius = 20
orbit_rate = 0.5
ch_time = 0.0

render_time = 0
t1 = time_module.time()

while ch_time < end_time:
    # Render scene
    if render_time > 0:
        manager.Update()
        if (not vis):
            print("Time = ":, ch_time, " Update rate = ":, manager.GetUpdateStep() / manager.GetSimStep())
        vehicle.GetVehicle().EnableRealtime(True)

    # End rendering scene

    # Update modules (process inputs from other modules)
    # Will spin the orbit
    time = vehicle.GetSystem().GetChTime()
    driver.Synchronize(time)
    driver.Inputs = driver_inputs
    driver.Update()

    # Simulation of vehicle (including powertrain and chassis)

    # Get driver inputs
    driver_inputs = driver.GetInputs()

    # Update vehicle
    vehicle.Synchronize(time)
    vehicle.Inputs = driver_inputs
    vehicle.Update()

    # Simulation of terrain
    terrain.Synchronize(time)
    terrain.Update()

    # Advance simulation for one timestep for all modules
    driver.Advance(step_size)
    vehicle.Advance(step_size)
    terrain.Advance(step_size)

    # Update sensor manager
    # Will render/save/filter automatically
    manager.Update()

    # Get the current time of the simulation
    ch_time = vehicle.GetSystem().GetChTime()

    if vis:
        print("Time = ":, ch_time, " Update rate = ":, manager.Get
print("error happened with only start ```python")