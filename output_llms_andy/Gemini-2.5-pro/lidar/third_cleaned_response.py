import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import os 







CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR', '../../../chrono_data/')
CHRONO_VEHICLE_DATA_DIR = os.environ.get('CHRONO_VEHICLE_DATA_DIR', '../../../chrono_vehicle_data/')


noise_model = "NONE"  
return_mode = sens.LidarReturnMode_STRONGEST_RETURN
update_rate = 5.0
horizontal_samples = 800
vertical_samples_3d = 30 
horizontal_fov = 2 * chrono.CH_PI  
max_vert_angle = chrono.CH_PI / 12.0
min_vert_angle = -chrono.CH_PI / 6.0
lag = 0.0
collection_time = 1.0 / update_rate
sample_radius = 1 
divergence_angle = 0.003


camera_update_rate = 30.0
camera_fov = chrono.CH_PI / 3.0
camera_width = 1280
camera_height = 720


step_size = 1e-3
end_time = 20.0 


vis = True 



def main():
    
    
    
    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    veh.SetDataPath(CHRONO_VEHICLE_DATA_DIR)

    
    
    
    my_vehicle = veh.ARTcar_Vehicle()
    my_vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    my_vehicle.SetChassisFixed(False)
    
    my_vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    my_vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    my_vehicle.SetBrakeType(veh.BrakeType_SHAFTS) 
    my_vehicle.SetTireType(veh.TireModelType_TMEASY) 
    my_vehicle.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(0, -2, 0.5), chrono.ChQuaterniond(1, 0, 0, 0))) 
    my_vehicle.Initialize()

    
    my_vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    my_vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    my_vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)

    mphysicalSystem = my_vehicle.GetSystem()
    mphysicalSystem.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))


    
    
    
    
    driver_data = veh.vector_ChDataDriverEntry()
    driver_data.push_back(veh.ChDataDriverEntry(0.0, 0.0, 0.2, 0.0))  
    driver_data.push_back(veh.ChDataDriverEntry(end_time, 0.0, 0.2, 0.0)) 
    my_driver = veh.ChDataDriver(my_vehicle.GetVehicle(), driver_data)
    my_driver.Initialize()
    
    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    
    patch = terrain.AddPatch(patch_mat, chrono.ChVector3d(0, 0, -0.1), chrono.ChVector3d(0,0,1), 200.0, 200.0, 0.1) 
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200) 
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()


    
    
    
    side = 4.0
    box = chrono.ChBodyEasyBox(side, side, side, 1000) 
    box.SetPos(chrono.ChVector3d(10, 0, side / 2.0)) 
    
    if box.GetVisualModel() and len(box.GetVisualModel().GetShapes()) > 0:
        try:
            box.GetVisualModel().GetShapes()[0].SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
        except Exception as e:
            print(f"Warning: Could not set texture for box: {e}")
    else:
        
        
        mat = chrono.ChVisualMaterial()
        mat.SetDiffuseColor(chrono.ChColor(0.2, 0.3, 0.8)) 
        mat.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))
        if box.GetVisualModel() and len(box.GetVisualModel().GetShapes()) > 0:
             box.GetVisualModel().GetShapes()[0].SetMaterial(0,mat)


    box.SetFixed(True)
    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    intensity = 1.0 
    manager.scene.AddPointLight(chrono.ChVector3d(100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(-100, 100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(100, -100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3d(-100, -100, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)


    
    
    
    
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0.0, 1.0), 
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )

    lidar_3d = sens.ChLidarSensor(
        my_vehicle.GetChassisBody(), 
        update_rate,
        lidar_offset_pose,
        horizontal_samples,
        vertical_samples_3d, 
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
    lidar_3d.SetName("3D Lidar Sensor")
    lidar_3d.SetLag(lag)
    lidar_3d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_3d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples_3d, "Raw 3D Lidar Depth Data"))
    lidar_3d.PushFilter(sens.ChFilterDIAccess())
    lidar_3d.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar_3d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar Point Cloud"))
    lidar_3d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_3d)

    
    
    
    
    lidar_2d = sens.ChLidarSensor(
        my_vehicle.GetChassisBody(), 
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
    if vis:
        
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "Raw 2D Lidar Depth Data"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    if vis: 
        lidar_2d.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 2.0, "2D Lidar Point Cloud"))
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-6.0, 0.0, 2.0), 
        chrono.QuatFromAngleAxis(chrono.CH_PI / 20.0, chrono.ChVector3d(0, 1, 0)) 
    )
    camera = sens.ChCameraSensor(
        my_vehicle.GetChassisBody(), 
        camera_update_rate,
        camera_offset_pose,
        camera_width,
        camera_height,
        camera_fov
    )
    camera.SetName("Third Person Camera")
    camera.SetLag(0) 
    camera.SetCollectionWindow(0) 
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(camera_width, camera_height, "Third Person Camera View"))
    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)
    
    
    
    
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        driver_inputs = my_driver.GetInputs()

        
        current_time = mphysicalSystem.GetChTime()
        my_driver.Synchronize(current_time)
        my_vehicle.Synchronize(current_time, driver_inputs.m_steering, driver_inputs.m_throttle, driver_inputs.m_braking, terrain)
        terrain.Synchronize(current_time)
        
        my_driver.Advance(step_size)
        my_vehicle.Advance(step_size)
        terrain.Advance(step_size)

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

        
        if lidar_3d.GetMostRecentXYZIBuffer().HasData():
            xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
            
            
            
            pass


    print("Sim time:", end_time, "Wall time:", time.time() - t1)

if __name__ == "__main__":
    main()