import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.irrlicht as irr

import numpy as np
import time
import math

# ---------------------------------------------------------------------
#
#  ARTcar vehicle setup
#
# ---------------------------------------------------------------------

# Create the vehicle system
vehicle_system = veh.ChVehicleSystemNSC()

# Create the vehicle
vehicle = veh.ChVehicle("My ARTcar", vehicle_system)

# Set vehicle parameters
vehicle.SetChassisFixed(False)
vehicle.SetInitWheelPos(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 0, 0))
vehicle.SetSteeringSharpen(1.0)
vehicle.SetTireType(veh.ChVehicle.TIRE_MODEL_RIGID)

# Create the driver
driver = veh.ChDriver(vehicle)

# Create the terrain
terrain = chrono.ChBodyEasyBox(100, 100, 0.1, 1000)
terrain.SetPos(chrono.ChVector3d(0, 0, -0.05))
terrain.SetBodyFixed(True)
terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/terrain.jpg"))
terrain.GetVisualModel().GetShape(0).SetColor(chrono.ChColor(0.8, 0.8, 0.8))
vehicle_system.Add(terrain)

# ---------------------------------------------------------------------
#
#  Sensor setup
#
# ---------------------------------------------------------------------

# Create a sensor manager
manager = sens.ChSensorManager(vehicle_system)

# ------------------------------------------------
# Create a lidar and add it to the sensor manager
# ------------------------------------------------
offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
lidar = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  # Attach lidar to vehicle chassis
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

# -----------------------------------------------------------------
# Create a filter graph for post-processing the data from the lidar
# -----------------------------------------------------------------
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
lidar_2d = sens.ChLidarSensor(
    vehicle.GetChassisBody(),  # Attach lidar to vehicle chassis
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
lidar_2d.
print("error happened with only start ```python")