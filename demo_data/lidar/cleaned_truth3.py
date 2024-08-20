import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.vehicle as veh

import numpy as np
import time
import math


def main():
    
    
    
    
    
    
    car = veh.ARTcar()
    car.SetContactMethod(chrono.ChContactMethod_NSC)
    car.SetChassisFixed(False)
    car.SetInitPosition(chrono.ChCoordsysd(initLoc, initRot))
    car.SetTireType(veh.TireModelType_TMEASY)
    car.SetTireStepSize(step_size)
    car.SetMaxMotorVoltageRatio(0.12)
    car.SetStallTorque(0.3)
    car.SetTireRollingResistance(0.06)

    car.Initialize()

    car.SetChassisVisualizationType(veh.VisualizationType_MESH)
    car.SetSuspensionVisualizationType(veh.VisualizationType_MESH)
    car.SetSteeringVisualizationType(veh.VisualizationType_MESH)
    car.SetWheelVisualizationType(veh.VisualizationType_MESH)
    car.SetTireVisualizationType(veh.VisualizationType_MESH)
    car.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    mphysicalSystem = car.GetSystem()
    
    
    driver = veh.ChDriver(car.GetVehicle())
    driver.Initialize()
    
    patch_mat = chrono.ChContactMaterialNSC()
    patch_mat.SetFriction(0.9)
    patch_mat.SetRestitution(0.01)
    terrain = veh.RigidTerrain(car.GetSystem())
    patch = terrain.AddPatch(patch_mat, 
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QUNIT), 
        20, 20)
    patch.SetTexture(veh.GetDataFile("terrain/textures/concrete.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()
    
    
    
    
    side = 4
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))
    box.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
    box.SetFixed(True)
    mphysicalSystem.Add(box)
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    lidar = sens.ChLidarSensor(
        car.GetChassisBody(),              
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
        car.GetChassisBody(),           
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
    
    
    cam_pose_offset = chrono.ChFramed(
        chrono.ChVector3d(-2.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    )
    camera = sens.ChCameraSensor(
        car.GetChassisBody(),              
        update_rate,            
        cam_pose_offset,        
        horizontal_samples,     
        vertical_samples,       
        1.48,       
    )
    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(horizontal_samples, horizontal_samples, "third person view"))

    manager.AddSensor(camera)
    
    
    

    ch_time = 0.0

    render_time = 0
    t1 = time.time()

    while ch_time < end_time:
        simtime = car.GetSystem().GetChTime()

        
        manager.Update()
        
        driver.SetSteering(0.0)
        driver.SetThrottle(0.0)
        driver_inputs = driver.GetInputs()
        
        driver.Synchronize(simtime)
        terrain.Synchronize(simtime)
        car.Synchronize(simtime, driver_inputs, terrain)
        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        car.Advance(step_size)
        
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



initLoc = chrono.ChVector3d(0, -5.0, 0.5)

initRot = chrono.QuatFromAngleZ(1.57)
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
main()