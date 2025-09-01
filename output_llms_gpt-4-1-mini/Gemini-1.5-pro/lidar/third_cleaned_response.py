import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import pychrono.assets as ass
import pychrono.irr as irr

import numpy as np
import time
import math








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






def main():
    
    
    
    system = chrono.ChSystemNSC()
    vehicle = veh.Vehicle(system, "ARTcar", "vehicle/artcar/json/artcar.json")
    vehicle.Initialize(chrono.ChCoordsD(chrono.ChVectorD(0, 0, 1), chrono.QUNIT))

    
    
    
    driver = veh.ChDriver(vehicle)
    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(system)
    patch = terrain.AddPatch(
        ass.ChBoxShape(500, 1, 500), chrono.ChVectorD(0, 0, -5), chrono.QUNIT, 10000
    )
    patch.SetContactFrictionCoefficient(0.9)
    patch.SetContactRestitutionCoefficient(0.01)
    patch.SetContactRollingFrictionCoefficient(0.01)
    patch.GetGroundBody().SetName("ground")
    patch.GetGroundBody().GetVisualModel().GetShape(0).SetTexture(
        chrono.GetChronoDataFile("textures/concrete.jpg")
    )
    patch.GetGroundBody().GetVisualModel().GetShape(0).SetTextureScale(200, 200)
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(system)

    
    
    
    
    chassis_body = vehicle.GetChassisBody()
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVectorD(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        chassis_body,           
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

    
    lidar_2d = sens.ChLidarSensor(
        chassis_body,           
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
        
        lidar_2d.PushFilter(
            sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data")
        )
    
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    
    manager.AddSensor(lidar_2d)

    
    
    
    
    camera = sens.ChCameraSensor(
        chassis_body,
        update_rate,
        chrono.ChFrameD(chrono.ChVectorD(-3, 0, 3), chrono.Q_from_AngAxis(0.2, chrono.ChVectorD(0, 1, 0))),
        640,
        480,
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    
    camera.PushFilter(sens.ChFilterRGBAccess())
    
    manager.AddSensor(camera)

    
    
    
    myapplication = irr.ChIrrApp(
        system, "Sensor Demonstration", irr.dimension2du(1280, 720), False, True
    )
    myapplication.SetTimestep(step_size)
    myapplication.SetTryRealtime(True)
    myapplication.SetBackground(
        chrono.ChColor(0.1, 0.1, 0.1), chrono.ChColor(0.9, 0.9, 0.9), True
    )
    myapplication.AssetBindAll()
    myapplication.AssetUpdateAll()
    myapplication.AddTypicalLights()
    myapplication.AddTypicalCamera(
        irr.vector3df(0.0, 4.0, -6.0), irr.vector3df(0, 1, 0)
    )

    ch_time = 0.0
    render_time = 0
    t1 = time.time()
    while ch_time < end_time:
        
        manager.Update(system.GetChTime())

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver_inputs, terrain)
        
        driver.Advance(step_size)
        
        system.DoStepDynamics(step_size)

        
        myapplication.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        myapplication.DrawAll()
        myapplication.EndScene()

        
        ch_time = system.GetChTime()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                "XYZI buffer received from lidar. Lidar resolution: {0}x{1}".format(
                    xyzi_buffer.Width, xyzi_buffer.Height
                )
            )
            print("Max Value: {0}".format(np.max(xyzi_data)))

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()