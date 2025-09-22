import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    # Create the system with ARTcar vehicle
    # Initialize the vehicle with given parameters
    vehicle = chrono.ChVehicleART()
    vehicle.SetName("ARTcar")
    vehicle.SetMass(1000)  # Mass of the vehicle
    vehicle.SetInertia(chrono.ChVector3d(0, 0, 100))  # Inertia
    vehicle.SetWidth(0.5)  # Width of the vehicle
    vehicle.SetHeight(1.0)  # Height of the vehicle
    vehicle.SetHeightOffset(1.0)  # Offset height for the vehicle
    vehicle.SetWheelbase(2.0)  # Wheelbase of the vehicle
    vehicle.SetSuspension(1.0)  # Suspension length
    vehicle.SetDamping(0.5, 0.5)  # Damping parameters
    vehicle.SetFriction(0.1)  # Friction coefficient
    vehicle.SetMaxTorque(100)  # Max torque
    vehicle.SetMaxSpeed(10.0)  # Max speed
    vehicle.SetSteeringRatio(0.5)  # Steering ratio
    vehicle.SetDriver(driver=chrono.ChDriverDefault())  # Initialize vehicle driver
    mphysicalSystem = vehicle  # Use the vehicle as the main system

    # Create a rigid terrain
    terrain = chrono.ChTerrain()
    terrain.SetMaterial(chrono.ChMaterial(chrono.ChMaterialType.Terrain, 1.0, 1.0, 1.0))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/ground.png"))
    terrain.SetPos(chrono.ChVector3d(0, 0, 0))
    mphysicalSystem.Add(terrain)

    # Create lidar sensors
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1),  # Offset pose position
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))  # Offset pose rotation
    )
    
    # 3D Lidar Sensor
    lidar_3d = sens.ChLidarSensor(
        vehicle,  # Lidar attached to vehicle chassis
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        vertical_samples,  # Number of vertical channels
        horizontal_fov,  # Horizontal field of view
        max_vert_angle,  # Maximum vertical field of view
        min_vert_angle,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
    )
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    # 2D Lidar Sensor
    lidar_2d = sens.ChLidarSensor(
        vehicle,  # Lidar attached to vehicle chassis
        update_rate,  # Scanning rate in Hz
        offset_pose,  # Offset pose
        horizontal_samples,  # Number of horizontal samples
        1,  # Only 1 vertical channel for 2D lidar
        horizontal_fov,  # Horizontal field of view
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,  # Shape of the lidar beam
        sample_radius,  # Sample radius
        divergence_angle,  # Divergence angle
        divergence_angle,  # Divergence angle (again, typically same value)
        return_mode  # Return mode for the lidar
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    # Add noise model filter if needed
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    elif noise_model == "NONE":
        pass

    # Add visualization filters if needed
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 3D Lidar Depth Data"))
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    
    # Provide access to depth and intensity data
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())

    # Add lidar sensors to sensor manager
    manager.AddSensor(lidar_3d)
    manager.AddSensor(lidar_2d)

    # Add third person camera sensor
    camera = sens.ChSensorCamera(
        vehicle,  # Attached to vehicle chassis
        "Third Person Camera",  # Sensor name
        0,  # Position x
        5,  # Position y
        10,  # Position z
        0,  # Rotation x
        0,  # Rotation y
        0,  # Rotation z
        45,  # Field of view
        1,  # Aspect ratio
        1000  # Near plane
    )
    camera.SetName("Third Person Camera")
    manager.AddSensor(camera)

    # Simulate system
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    end_time = 40.0
    step_size = 1e-3
    t1 = time.time()

    while ch_time < end_time:
        # Update vehicle and terrain
        vehicle.Update()
        terrain.Update()
        
        # Set lidar offset pose
        lidar_3d.SetOffsetPose(offset_pose)
        lidar_2d.SetOffsetPose(offset_pose)

        # Access data from lidar
        xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('3D Lidar buffer received. Resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        # Update sensor manager and perform dynamics
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)

        # Update simulation time
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# Lidar parameters
noise_model = "NONE"  # No noise model
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI  # 360 degrees
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1. / update_rate  # Typically 1/update rate
sample_radius = 2
divergence_angle = 0.003

# Simulation parameters
step_size = 1e-3
end_time = 40.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"