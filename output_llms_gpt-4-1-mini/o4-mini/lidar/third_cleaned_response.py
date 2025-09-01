import pychrono.core as chrono
import pychrono.vehicle as veh
import pychrono.sensor as sens

import numpy as np
import time
import math

def main():
    
    
    
    
    
    
    veh.SetChronoDataPath(chrono.GetChronoDataPath() + "vehicle/")

    
    
    
    
    artcar = veh.WheeledVehicle(
        veh.GetDataFile("generic/vehicle/ARTcar/vehicle/ARTcar.json"),
        chrono.ChContactMethod_NSC
    )
    
    init_loc = chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)
    artcar.Initialize(init_loc)
    
    sys = artcar.GetSystem()

    
    
    
    driver = veh.ChDriver(artcar)
    driver.Initialize()

    
    
    
    terrain = veh.RigidTerrain(sys)
    
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.9)
    mat.SetRestitution(0.01)
    terrain.SetContactSurface(material=mat)
    
    terrain.SetPlane(chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT))
    
    terrain.SetTexture(chrono.GetChronoDataFile("terrain/textures/tile4.jpg"), 10, 10)
    terrain.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    
    
    
    manager = sens.ChSensorManager(sys)

    
    
    
    
    lidar_offset = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0.0, 1.0),
        chrono.QUNIT
    )
    lidar = sens.ChLidarSensor(
        artcar.GetChassisBody(),  
        update_rate,              
        lidar_offset,
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
    lidar.SetName("3D Lidar")
    lidar.SetLag(lag)
    lidar.SetCollectionWindow(collection_time)
    
    if noise_model == "CONST_NORMAL_XYZI":
        lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    
    if vis:
        lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples, "3D Lidar Depth"))
    lidar.PushFilter(sens.ChFilterDIAccess())
    lidar.PushFilter(sens.ChFilterPCfromDepth())
    if vis:
        lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0, "3D Lidar PC"))
    lidar.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar)

    
    
    
    lidar2_offset = chrono.ChFramed(
        chrono.ChVector3d(1.0, 0.0, 1.0),
        chrono.QUNIT
    )
    lidar_2d = sens.ChLidarSensor(
        artcar.GetChassisBody(),
        update_rate,
        lidar2_offset,
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
    lidar_2d.SetName("2D Lidar")
    lidar_2d.SetLag(lag)
    lidar_2d.SetCollectionWindow(collection_time)
    if noise_model == "CONST_NORMAL_XYZI":
        lidar_2d.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))
    if vis:
        lidar_2d.PushFilter(sens.ChFilterVisualize(horizontal_samples, 1, "2D Lidar Depth"))
    lidar_2d.PushFilter(sens.ChFilterDIAccess())
    lidar_2d.PushFilter(sens.ChFilterPCfromDepth())
    lidar_2d.PushFilter(sens.ChFilterXYZIAccess())
    manager.AddSensor(lidar_2d)

    
    
    
    cam_offset = chrono.ChFramed(
        chrono.ChVector3d(-5.0, 0.0, 2.0),  
        chrono.ChQuaternionD(1, 0, 0, 0)     
    )
    chase_cam = sens.ChCameraSensor(
        artcar.GetChassisBody(),
        update_rate,
        cam_offset,
        800,    
        600,    
        0.1,    
        1000.0  
    )
    chase_cam.SetName("ChaseCam")
    chase_cam.SetLag(0)
    chase_cam.SetCollectionWindow(1.0 / update_rate)
    if vis:
        chase_cam.PushFilter(sens.ChFilterVisualize(800, 600, "Chase Camera"))
    manager.AddSensor(chase_cam)

    
    
    
    ch_time = 0.0
    t_start = time.time()

    while ch_time < end_time:
        
        driver.Synchronize(ch_time)
        terrain.Synchronize(ch_time)
        artcar.Synchronize(ch_time, driver.GetInputs(), terrain)
        manager.Update()

        
        sys.DoStepDynamics(step_size)
        driver.Advance(step_size)
        terrain.Advance(step_size)
        artcar.Advance(step_size)

        
        ch_time = sys.GetChTime()

        
        buf = lidar.GetMostRecentXYZIBuffer()
        if buf.HasData():
            xyz = buf.GetXYZIData()
            print(
                "t={:.2f}: 3D Lidar {}×{}, max range {:.3f}".format(
                    ch_time, buf.Width, buf.Height, np.max(xyz[:, 3])
                )
            )

    print("Sim end time:", end_time, "Elapsed wall time:", time.time() - t_start)





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




step_size = 1e-3
end_time = 40.0
vis = True

if __name__ == "__main__":
    main()