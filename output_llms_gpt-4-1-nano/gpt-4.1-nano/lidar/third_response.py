import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens
import numpy as np
import time
import math

# --------------------------
# Simulation & Sensor Parameters
# --------------------------
# Lidar parameters
# Noise model attached to the sensor
# TODO: Noise models haven't been implemented in python
# noise_model = "CONST_NORMAL_XYZI"  # Gaussian noise with constant mean and std dev
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
# return_mode = sens.MEAN_RETURN
# return_mode = sens.FIRST_RETURN
# return_mode = sens.LAST_RETURN

# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 800
vertical_samples = 300

# Horizontal and vertical field of view (radians)
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6

# Lag time
lag = 0

# Collection window for the lidar
collection_time = 1. / update_rate  # typically 1/update_rate

# Radius of samples to use
sample_radius = 2

# Divergence angle
divergence_angle = 0.003

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"

# --------------------------
# System Initialization with Vehicle
# --------------------------

# Create the Chrono vehicle system
my_system = chrono.ChSystemNSC()

# Create and initialize terrain
terrain_material = veh.ChMaterialSurfaceNSC()
terrain_material.SetFriction(0.8)
terrain_material.SetCollide(true)

# Create a rigid terrain
terrain_body = chrono.ChBodyEasyTerrain(chrono.GetChronoDataFile("terrain/terrain_2.tn"), 1000, True, terrain_material)
terrain_body.SetPos(chrono.ChVector3d(0, 0, 0))
terrain_body.SetBodyFixed(True)
my_system.Add(terrain_body)

# Create the vehicle (ARTcar)
my_vehicle = veh.ChWheeledVehicle(veh.CHWN_TYPE::articulated, my_system)
# Load vehicle config (assuming default setup, replace with actual if needed)
vehicle_params = veh.ChVehicleAssemblyLine()
vehicle_params.Create(my_vehicle, "Artcar", veh.ChVehicleAssemblyLine.VEHICLE_TYPE::ARTICULATED)
# Set initial position
my_vehicle.GetChassis().SetPos(chrono.ChVector3d(0, 0, 0.5))
# Initialize the vehicle
my_vehicle.Initialize(chrono.ChVectorD(0, 0, 0.5))

# Create driver (default settings)
driver = veh.ChIrrGuiDriver()
driver.Initialize(my_vehicle)

# --------------------------
# Create Sensor Manager
# --------------------------
manager = sens.ChSensorManager(my_system)

# --------------------------
# Attach Lidar Sensors to Vehicle Chassis
# --------------------------

# Create a transformation for the lidar offset pose
lidar_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
)

# Create 3D lidar sensor attached to vehicle
lidar_body = my_vehicle.GetChassis()
lidar = sens.ChLidarSensor(
    lidar_body,
    update_rate,
    lidar_offset_pose,
    horizontal_samples,
    vertical_samples,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    return_mode
)
lidar.SetName("Lidar Sensor")
lidar.SetLag(lag)
lidar.SetCollectionWindow(collection_time)

# Post-processing filters for lidar
if noise_model == "CONST_NORMAL_XYZI":
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
elif noise_model == "NONE":
    pass
if vis:
    lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw Lidar Depth Data"))
lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())
if vis:
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "Lidar Point Cloud"))
lidar.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar)

# Create a 2D lidar attached to vehicle
lidar_2d = sens.ChLidarSensor(
    lidar_body,
    update_rate,
    lidar_offset_pose,
    horizontal_samples,
    1,
    horizontal_fov,
    0.0,
    0.0,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    return_mode
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(lag)
lidar_2d.SetCollectionWindow(collection_time)
if noise_model == "CONST_NORMAL_XYZI":
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
elif noise_model == "NONE":
    pass
if vis:
    lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
lidar_2d.PushFilter(sens.ChFilterDIAccess())
lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
manager.AddSensor(lidar_2d)

# --------------------------
# Add a third-person camera sensor attached to vehicle
# --------------------------
cam_offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0.0, -5.0, 2.0),
    chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 0, 1))
)
camera_sensor = sens.ChCameraSensor(
    lidar_body,
    update_rate,
    cam_offset_pose,
    640,
    480,
    60.0,
    0,
    1000
)
camera_sensor.SetName("Third Person Camera")
manager.AddSensor(camera_sensor)

# --------------------------
# Simulation Loop
# --------------------------

t1 = time.time()
ch_time = 0

while ch_time < end_time:
    # Update driver inputs
    driver.ComputeCommands()
    my_vehicle.GetDrvBase().Synchronize(ch_time)
    my_vehicle.GetChassis().Synchronize(ch_time)
    my_vehicle.GetPowertrain().Synchronize(ch_time)
    my_vehicle.GetDriveline().Synchronize(ch_time)
    my_vehicle.GetSteering().Synchronize(ch_time)
    my_vehicle.GetSuspensions().Synchronize(ch_time)
    my_vehicle.GetWheels().Synchronize(ch_time)

    # Update terrain if dynamic (here static terrain, so no sync needed)
    # Synchronize vehicle (simulate)
    my_vehicle.Advance(step_size)
    
    # Move lidar around vehicle if needed (currently attached to chassis)
    # If moving lidar independently, update offset pose here

    # Access and print from lidar buffer
    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
    if xyzi_buffer.HasData():
        xyzi_data = xyzi_buffer.GetXYZIData()
        print('XYZI buffer received from lidar. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
        print('Max Value: {0}'.format(np.max(xyzi_data)))

    # Update sensor manager
    manager.Update()

    # Advance the physical system
    my_system.DoStepDynamics(step_size)

    # Increment time
    ch_time = my_system.GetChTime()

print("Simulation completed in wall time: ", time.time() - t1)