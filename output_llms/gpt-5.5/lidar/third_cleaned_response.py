import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import numpy as np
import time
import math
import os








noise_model = "NONE"



return_mode = sens.LidarReturnMode_STRONGEST_RETURN


update_rate = 5.0


horizontal_samples = 800
vertical_samples = 300


horizontal_fov = 2 * chrono.CH_PI
max_vert_angle = chrono.CH_PI / 12
min_vert_angle = -chrono.CH_PI / 6


lag = 0.0


collection_time = 1.0 / update_rate


sample_radius = 2


divergence_angle = 0.003





camera_update_rate = 30.0
camera_width = 1280
camera_height = 720
camera_fov = chrono.CH_PI / 3.0





contact_method = chrono.ChContactMethod_NSC
chassis_collision_type = veh.CollisionType_NONE
tire_model = veh.TireModelType_RIGID

init_loc = chrono.ChVector3d(0.0, 0.0, 0.5)
init_rot = chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)





step_size = 1e-3
tire_step_size = 1e-3
end_time = 40.0

save = False
vis = True

out_dir = "SENSOR_OUTPUT/"


def main():
    
    
    
    veh.SetDataPath(chrono.GetChronoDataPath() + "vehicle/")

    if save and not os.path.exists(out_dir):
        os.makedirs(out_dir)

    
    
    
    vehicle = veh.ARTcar()
    vehicle.SetContactMethod(contact_method)
    vehicle.SetChassisCollisionType(chassis_collision_type)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(init_loc, init_rot))
    vehicle.SetTireType(tire_model)
    vehicle.SetTireStepSize(tire_step_size)

    vehicle.Initialize()

    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)

    
    mphysicalSystem = vehicle.GetSystem()

    
    chassis_body = vehicle.GetChassisBody()

    
    
    
    driver = veh.ChDriver(vehicle.GetVehicle())
    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(mphysicalSystem)

    terrain_mat = chrono.ChContactMaterialNSC()
    terrain_mat.SetFriction(0.9)
    terrain_mat.SetRestitution(0.01)

    patch = terrain.AddPatch(
        terrain_mat,
        chrono.ChCoordsysd(chrono.ChVector3d(0.0, 0.0, 0.0), chrono.ChQuaterniond(1.0, 0.0, 0.0, 0.0)),
        100.0,
        100.0,
    )
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 100.0, 100.0)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))

    terrain.Initialize()

    
    
    
    side = 4.0
    box = chrono.ChBodyEasyBox(side, side, side, 1000.0, True, True)
    box.SetPos(chrono.ChVector3d(15.0, 0.0, side / 2.0))
    box.SetFixed(True)

    try:
        box.GetVisualModel().GetShape(0).SetTexture(
            chrono.GetChronoDataFile("textures/blue.png")
        )
    except Exception:
        pass

    mphysicalSystem.Add(box)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    try:
        manager.scene.AddPointLight(
            chrono.ChVector3f(0.0, 0.0, 20.0),
            chrono.ChColor(1.0, 1.0, 1.0),
            500.0,
        )
        manager.scene.AddAmbientLight(chrono.ChColor(0.3, 0.3, 0.3))
    except Exception:
        pass

    
    
    
    lidar_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0.0, 1.0),
        chrono.QuatFromAngleAxis(0.0, chrono.ChVector3d(0.0, 1.0, 0.0)),
    )

    lidar = sens.ChLidarSensor(
        chassis_body,                         
        update_rate,                          
        lidar_offset_pose,                    
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
        return_mode,                          
    )

    lidar.SetName("3D Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples,
                vertical_samples,
                "Raw 3D Lidar Depth Data",
            )
        )

    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640,
                480,
                1.0,
                "3D Lidar Point Cloud",
            )
        )

    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    lidar_2d = sens.ChLidarSensor(
        chassis_body,                         
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
        return_mode,                          
    )

    lidar_2d.SetName("2D Lidar Sensor")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)

    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))

    if vis:
        
        lidar_2d.PushFilter(
            sens.ChFilterVisualize(
                horizontal_samples,
                1,
                "Raw 2D Lidar Depth Data",
            )
        )

    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())

    if vis:
        lidar_2d.PushFilter(
            sens.ChFilterVisualizePointCloud(
                640,
                480,
                1.0,
                "2D Lidar Point Cloud",
            )
        )

    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    camera_pitch = math.atan2(3.0, 6.0)

    camera_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-6.0, 0.0, 3.0),
        chrono.QuatFromAngleAxis(camera_pitch, chrono.ChVector3d(0.0, 1.0, 0.0)),
    )

    camera = sens.ChCameraSensor(
        chassis_body,
        camera_update_rate,
        camera_offset_pose,
        camera_width,
        camera_height,
        camera_fov,
    )

    camera.SetName("Third Person Camera")
    camera.SetLag(0.0)
    camera.SetCollectionWindow(1.0 / camera_update_rate)

    if vis:
        camera.PushFilter(
            sens.ChFilterVisualize(
                camera_width,
                camera_height,
                "Third Person Camera",
            )
        )

    if save:
        camera_out_dir = os.path.join(out_dir, "third_person_camera")
        os.makedirs(camera_out_dir, exist_ok=True)
        camera.PushFilter(sens.ChFilterSave(camera_out_dir + "/"))

    camera.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(camera)

    
    
    
    t1 = time.time()

    while mphysicalSystem.GetChTime() < end_time:
        ch_time = mphysicalSystem.GetChTime()

        
        driver_inputs = driver.GetInputs()

        
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        vehicle.Synchronize(ch_time, driver_inputs, terrain)

        
        manager.Update()

        
        xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
        if xyzi_buffer.HasData():
            xyzi_data = xyzi_buffer.GetXYZIData()
            print(
                "XYZI buffer received from 3D lidar. "
                "Resolution: {0}x{1}".format(xyzi_buffer.Width, xyzi_buffer.Height)
            )
            print("3D lidar max value:", np.max(xyzi_data))

        
        xyzi_buffer_2d = lidar_2d.GetMostRecentXYZIBuffer()
        if xyzi_buffer_2d.HasData():
            xyzi_data_2d = xyzi_buffer_2d.GetXYZIData()
            print(
                "XYZI buffer received from 2D lidar. "
                "Resolution: {0}x{1}".format(xyzi_buffer_2d.Width, xyzi_buffer_2d.Height)
            )
            print("2D lidar max value:", np.max(xyzi_data_2d))

        
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == "__main__":
    main()