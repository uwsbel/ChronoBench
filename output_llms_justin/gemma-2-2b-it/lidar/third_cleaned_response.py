import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import math






mphysicalSystem = chrono.ChSystemNSC()


artcar = chrono.ChARTCar(mphysicalSystem)
artcar.SetPos(chrono.ChVector3d(0, 0, 0))
artcar.GetVisualModel().GetShape(0).SetTexture(chrono.GetChronoDataFile("textures/blue.png"))
artcar.SetFixed(True)
mphysicalSystem.Add(artcar)


driver = chrono.ChDriver()
driver.SetVehicle(artcar)
driver.SetVehicleControlMode(chrono.ChVehicleControlMode_AUTOMATIC)
mphysicalSystem.Add(driver)


terrain = chrono.ChTerrain(chrono.ChVector3d(0, 0, 0), 10, 10, 10)
terrain.SetMaterial(chrono.ChMaterial(chrono.ChMaterialType_GRASS, 0.2, 0.2, 0.2))
terrain.SetTexture(chrono.GetChronoDataFile("textures/grass.png"))
mphysicalSystem.Add(terrain)





offset_pose = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
lidar_3d = sens.ChLidarSensor(
    artcar,
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
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_3d.SetName("3D Lidar Sensor")
lidar_3d.SetLag(lag)
lidar_3d.SetCollectionWindow(collection_time)


offset_pose_2d = chrono.ChFramed(
    chrono.ChVector3d(1.0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
)
lidar_2d = sens.ChLidarSensor(
    artcar,
    update_rate,
    offset_pose_2d,
    horizontal_samples,
    1,
    horizontal_fov,
    max_vert_angle,
    min_vert_angle,
    100.0,
    sens.LidarBeamShape_RECTANGULAR,
    sample_radius,
    divergence_angle,
    divergence_angle,
    sens.LidarReturnMode_STRONGEST_RETURN,
)
lidar_2d.SetName("2D Lidar Sensor")
lidar_2d.SetLag(lag)
lidar_2d.SetCollectionWindow(collection_time)




manager = sens.ChSensorManager(mphysicalSystem)


manager.AddSensor(lidar_3d)
manager.AddSensor(lidar_2d)




orbit_radius = 10
orbit_rate = 0.1
ch_time = 0.0

render_time = 0
t1 = time.time()

while ch_time < end_time:
    
    lidar_3d.SetOffsetPose(
        chrono.ChFramed(
            chrono.ChVector3d(
                -orbit_radius * math.cos(ch_time * orbit_rate),
                -orbit_radius * math.sin(ch_time * orbit_rate),
                1
            ),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        )
    )

    
    xyzi_buffer = lidar_3d.GetMostRecentXYZIBuffer()
    if xyzi_buffer.HasData():
        xyzi_data = xyzi_buffer.GetXYZIData()
        print('XYZI buffer received from lidar. Lidar resolution: {0}x{1}'.format(xyzi_buffer.Width, xyzi_buffer.Height))
        print('Max Value: {0}'.format(np.max(xyzi_data)))

    
    manager.Update()

    
    mphysicalSystem.DoStepDynamics(step_size)

    
    ch_time = mphysicalSystem.GetChTime()

    
    
    
    if vis:
        
        manager.Render()

    
    
    
    if save:
        
        manager.Save()

    
    
    
    render_time = time.time()


print("Sim time:", end_time, "Wall time:", time.time() - t1)