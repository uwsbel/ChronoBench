```python
import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()
    
    # Create ARTcar vehicle
    vehicle_model = vehicle.ChVehicleModel()
    vehicle_model.SetChassis(vehicle.ChVehicleChassis(vehicle_model))
    vehicle_model.SetWheels(
        [
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
            vehicle.ChVehicleWheel(vehicle_model, 0.2, 0.1, 0.1),
        ]
    )
    
    vehicle_system = vehicle.ChVehicleSystem(mphysicalSystem, vehicle_model)
    
    # Create driver
    driver = vehicle.ChVehicleDriver(vehicle_system)
    driver.SetDriverType(vehicle.ChVehicleDriverType_PID)
    driver.SetController(vehicle.ChVehicleController_PID(
        kp=1.0, ki=0.1, kd=0.01
    ))
    
    # Create terrain
    terrain = chrono.ChBodyEasyBox(100, 100, 1, 1000)
    terrain.SetPos(chrono.ChVector3d(0, 0, -1))
    terrain.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    terrain.SetBodyFixed(True)
    mphysicalSystem.Add(terrain)
    
    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle_system.GetChassis(),              # Body lidar is attached to
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
    manager.AddSensor(lidar)

    # Create 2D lidar sensor
    lidar_2d =  sens.ChLidarSensor(
        vehicle_system.GetChassis(),              # Body lidar is attached to
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
        lidar_2d.PushFilter(sens