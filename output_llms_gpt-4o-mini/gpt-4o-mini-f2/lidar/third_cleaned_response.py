import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh
import numpy as np
import time
import math


def main():
    
    
    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(chrono.ChContactMethod_NSC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    vehicle.SetTireType(veh.TireModelType_TMEASY)
    vehicle.SetTireStepSize(step_size)
    vehicle.SetChassisMass(450.0)
    vehicle.SetEngineType(veh.EngineModelType_SIMPLE)
    vehicle.SetTransmissionType(veh.TransmissionModelType_SIMPLE)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetSteeringType(veh.SteeringTypePS)
    vehicle.SetBrakeType(veh.BrakeType_SIMPLE)
    vehicle.SetSuspensionType(veh.SuspensionTypeDoubleWishbone)
    vehicle.SetWheelType(veh.WheelTypeSpherical())
    vehicle.Initialize()
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    mphysicalSystem = vehicle.GetSystem()
    
    vehicle.SetInitialSpeed(10)
    vehicle.Initialize()
    vehicle.SetChassisFixed(True)
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)
    
    driver = veh.ChDriver(vehicle.GetChassis())
    driver.Initialize()
    
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(vehicle.GetSystem())
    patch = terrain.AddPatch(patch_mat, chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT),
                              10, 10)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    terrain.Initialize()
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        vehicle.GetChassisBody(),              
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
        vehicle.GetChassisBody(),              
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
    
    
    cam_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-2.0, -2.0, 2.0), chrono.QuatFromAngleAxis(-math.pi / 4, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        vehicle.GetChassisBody(),            
        update_rate,            
        cam_offset_pose,        
        horizontal_samples,      
        vertical_samples,        
        1.0,                    
    )
    camera.SetName("Third Person POV")
    camera.SetLag(lag)
    camera.SetCollectionWindow(collection_time)
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "Third Person POV"))
    manager.AddSensor(camera)
    
    
    
    orbit_radius = 10
    orbit_rate = 0.1
    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
            print('Max Value: {0}'.format(np.max(xyzi_data)))

        
        manager.Update()
        
        driver_inputs = driver.GetInputs()
        
        terrain.Sync(ch_time)
        patch.Sync(ch_time)
        vehicle.Sync(ch_time, driver_inputs, terrain)
        driver.Sync(ch_time, driver_inputs)
        
        terrain.Advance(step_size)
        patch.Advance(step_size)
        vehicle.Advance(step_size)
        driver.Advance(step_size)
        
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


initLoc = chrono.ChVector3d(0, 0, 0.5)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)





main()