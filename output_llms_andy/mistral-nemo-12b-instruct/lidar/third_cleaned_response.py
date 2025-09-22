import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as vehicle
import pychrono.irrlicht as chronoirr
import numpy as np
import time
import math

def main():
    
    
    
    
    artcar = vehicle.ChArtCar()
    artcar.SetChassisFixed(False)
    artcar.SetChassisBuoyancy(False)
    artcar.SetChassisSuspensionDamping(0.2)
    artcar.SetChassisSuspensionStiffness(20000)
    artcar.SetChassisWheelFriction(0.5)
    artcar.SetChassisWheelRadius(0.2)
    artcar.SetChassisWheelWidth(0.1)
    artcar.SetChassisWheelSeparation(1.0)
    artcar.SetChassisWheelToChassisDistance(0.5)
    artcar.SetChassisWheelToGroundDistance(0.1)
    artcar.SetChassisWheelSteeringAngle(0.0)
    artcar.SetChassisWheelSteeringRate(0.0)
    artcar.SetChassisWheelDriveTorque(0.0)
    artcar.SetChassisWheelBrakeTorque(0.0)

    mphysicalSystem = artcar.GetSystem()
    mphysicalSystem.Set_Gravity(chrono.ChVectorD(0, 0, -9.81))

    
    
    
    driver = vehicle.ChChassisDriver()
    artcar.AddDriver(driver)

    
    
    
    terrain = chrono.ChBody()
    terrain.SetBodyFixed(True)
    terrain.SetPos(chrono.ChVectorD(0, 0, -1))
    terrain.SetMaterialSurface(chrono.ChMaterialSurfaceNSC())
    terrain.GetMaterialSurface().SetFriction(0.6)
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.8))
    terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
    mphysicalSystem.Add(terrain)

    
    
    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVectorD(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),              
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
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    
    
    
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

    
    lidar_2d =  sens.ChLidarSensor(
        artcar.GetChassisBody(),              
        update_rate,            
        offset_pose,            
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
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Raw 2D Lidar Depth Data"))
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    camera = chronoirr.ChIrrNodeCamera()
    camera.SetChassisBody(artcar.GetChassisBody())
    camera.SetName("Camera")
    camera.LookAt(chrono.ChVectorD(0, 0, 0))
    camera.SetFOV(chrono.CH_C_PI/4)
    mphysicalSystem.Add(camera)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        artcar.Synchronize()
        driver.Advance()
        terrain.Advance()

        
        lidar.SetOffsetPose(
            chrono.ChFramed(
                chrono.ChVectorD(
                    -orbit_radius * math.cos(ch_time * orbit_rate),
                    -orbit_radius * math.sin(ch_time * orbit_rate),
                    1
                ),
                chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)








noise_model = "NONE"  


return_mode = sens.LidarReturnMode_STRONGEST_RETURN





update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0


collection_time = 1. / update_rate  


sample_radius = 2


divergence_angle = 0.003






step_size = 1e-3


end_time = 40.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"






main()