import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    # Initialize ARTcar vehicle
    vehicle = chrono.ChVehicleART()
    vehicle.SetName("ARTcar")
    
    # Set vehicle parameters
    vehicle.SetInertia(chrono.ChVector3d(1000, 1000, 1000))
    vehicle.SetMass(1000)
    vehicle.SetWidth(0.5)
    vehicle.SetHeight(1.0)
    vehicle.SetWheelbase(1.2)
    vehicle.SetSuspension(0.5)
    vehicle.SetDamping(0.1, 0.1)
    vehicle.SetSteering(0.5)
    vehicle.SetEngine(100.0, 0.1)
    vehicle.SetMotor(100.0, 0.1)
    
    # Set up the main physical system
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.Add(vehicle)
    
    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    
    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    
    # 3D Lidar
    lidar = sens.ChLidarSensor(
        vehicle.GetChassis(),              # Body lidar is attached to
        10.0,                             # Update rate in Hz
        offset_pose,                        # Offset pose
        800,                                # Number of horizontal samples
        300,                                 # Number of vertical channels
        2 * chrono.CH_PI,                    # Horizontal field of view
        chrono.CH_PI / 12,                   # Maximum vertical field of view
        -chrono.CH_PI / 6,                   # Minimum vertical field of view
        100.0,                               # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,       # Shape of the lidar beam
        2,                                     # Sample radius
        0.003,                                # Divergence angle
        0.003,                                # Divergence angle (again, typically same value)
        sens.LidarReturnMode_STRONGEST_RETURN   # Return mode for the lidar
    )
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(0)
    lidar.SetCollectionWindow(1.0 / 10.0)  # Collection window
    
    # Add noise model if needed
    noise_model = "NONE"  # No noise model
    
    # Filter graph for 3D lidar
    lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01)) if noise_model == "CONST_NORMAL_XYZI" else None
    lidar.PushFilter(sens.ChFilterVisualize(800, 300, "Raw 3D Lidar Depth Data")) if vis else None
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar)
    
    # 2D Lidar
    lidar_2d = sens.ChLidarSensor(
        vehicle.GetChassis(),
        10.0,
        offset_pose,
        1,  # only 1 vertical channel for 2D lidar
        2 * chrono.CH_PI,  # Horizontal field of view
        0.0,  # Maximum vertical field of view
        0.0,  # Minimum vertical field of view
        100.0,  # Maximum lidar range
        sens.LidarBeamShape_RECTANGULAR,
        2,
        0.003,
        0.003,
        sens.LidarReturnMode_STRONGEST_RETURN
    )
    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(0)
    lidar_2d.SetCollectionWindow(1.0 / 10.0)
    
    # Filter graph for 2D lidar
    lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01)) if noise_model == "CONST_NORMAL_XYZI" else None
    lidar_2d.PushFilter(sens.ChFilterVisualize(800, 1, "Raw 2D Lidar Depth Data")) if vis else None
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)
    
    # -------------------------------
    # Add third person camera sensor
    # -------------------------------
    camera_offset = chrono.ChFramed(
        chrono.ChVector3d(0, 1.5, 1),
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    third_person_camera = sens.ChSensorCamera(
        vehicle.GetChassis(),
        0,
        camera_offset,
        640,
        480,
        1.0,
        "Perspective",
        45.0,
        0.1,
        1000.0
    )
    third_person_camera.SetName("Third Person Camera")
    manager.AddSensor(third_person_camera)
    
    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0
    end_time = 40.0
    
    render_time = 0
    t1 = time.time()
    
    while ch_time < end_time:
        # Update vehicle and driver
        vehicle.Update()
        driver = vehicle.GetDriver()
        driver.Update()
        
        # Set lidar to orbit around the vehicle
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVector3d(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
            )
        )
        
        # Access the XYZI buffer from lidar
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))
        
        # Update sensor manager (will render/save/filter automatically)
        manager.Update()
        
        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)
        
        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()
        
        # Render camera view if enabled
        if vis:
            render_time += 1.0 / 100.0
            if render_time >= 1.0:
                render_time = 0.0
                # Render the scene
                renderer = mphysicalSystem.GetRenderer()
                renderer.Render()
    
    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# Lidar parameters
noise_model = "NONE"  # No noise model

# Lidar return mode
return_mode = sens.LidarReturnMode_STRONGEST_RETURN

# Update rate in Hz
update_rate = 10.0

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
collection_time = 1. / update_rate  # typically 1/update rate

# Radius of samples to use, 1->1 sample, 2->9 samples, 3->25 samples...
sample_radius = 2

# 3mm radius (as cited by velodyne)
divergence_angle = 0.003

# Simulation parameters
step_size = 1e-3

# Simulation end time
end_time = 40.0

# Save camera images
save = False

# Render camera images
vis = True

# Output directory
out_dir = "SENSOR_OUTPUT/"