import pychrono.core as chrono
import pychrono.sensor as sens
import numpy as np
import time
import math

def main():
    # -----------------
    # Create the system
    # -----------------
    mphysicalSystem = chrono.ChSystemNSC()

    # ----------------------
    # Create a box body
    # ----------------------
    side = 2.0  # Box side length
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetFixed(True)
    # Set texture for visualization
    vis_shapes = box_body.GetVisualShapes()
    if vis_shapes:
        vis_shapes[0].SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    mphysicalSystem.Add(box_body)

    # -----------------------
    # Create sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)

    # ------------------------------------------------
    # Create a lidar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-12, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        box_body,              # Attached to box body
        update_rate,           
        offset_pose,           
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
    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    # -------------------------------------------------
    # Create a 2D lidar and add it to the sensor manager
    # -------------------------------------------------
    offset_pose_2d = chrono.ChFramed(
        chrono.ChVector3d(0, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar_2d = sens.ChLidarSensor(
        box_body,                # Attached to box body
        update_rate,             
        offset_pose_2d,          
        horizontal_samples,      # Same horizontal samples
        1,                       # Single vertical channel
        horizontal_fov,          
        0.0,                     # Max vertical angle
        0.0,                     # Min vertical angle (same as max for 2D)
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

    # -----------------------------------------------------------------
    # Create filter graphs for both lidars
    # -----------------------------------------------------------------
    for sensor in [lidar, lidar_2d]:
        if noise_model == "CONST_NORMAL_XYZI":
            sensor.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
        if vis:
            sensor.PushFilter(sens.ChFilterVisualize(sensor.GetNumHorizontalSamples(), 
                                                    sensor.GetNumVerticalChannels(), 
                                                    f"Raw Data - {sensor.GetName()}"))
        sensor.PushFilter(sens.ChFilterDIAccess())
        sensor.PushFilter(sens.ChFilterPCfromDepth())
        if vis:
            sensor.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, f"Point Cloud - {sensor.GetName()}"))
        sensor.PushFilter(sens.ChFilterXYZIAccess())

    manager.AddSensor(lidar)
    manager.AddSensor(lidar_2d)

    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        # Update 3D lidar orbit position
        lidar.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        ))

        # Access data from both sensors
        for sensor in [lidar, lidar_2d]:
            xyzi_buffer = sensor.GetMostRecentXYZIBuffer()
            if xyzi_buffer.HasData():
                print(f'{sensor.GetName()} resolution: {xyzi_buffer.Width}x{xyzi_buffer.Height}')

        # Update sensor manager
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)

# -----------------
# Sensor parameters
# -----------------
noise_model = "NONE"  
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples = 300
horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6
lag = 0
collection_time = 1.0 / update_rate
sample_radius = 2
divergence_angle = 0.003

# ---------------------
# Simulation parameters
# ---------------------
step_size = 1e-3
end_time = 40.0
vis = True

main()