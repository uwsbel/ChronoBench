import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time




chrono.SetChronoDataPath("data/")


mphysicalSystem = chrono.ChSystemNSC()







mmesh = chrono.ChTriangleMeshConnected()
mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
                        False, True)

mmesh.Transform(chrono.ChVectorD(0, 0, 0),
                chrono.ChMatrix33D(2))  


trimesh_shape = chrono.ChTriangleMeshShape()
trimesh_shape.SetMesh(mmesh)
trimesh_shape.SetName("OBJ_Mesh")
trimesh_shape.SetMutable(False)


mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.AddVisualShape(trimesh_shape)
mesh_body.SetBodyFixed(True)   
mphysicalSystem.Add(mesh_body)




manager = sens.ChSensorManager(mphysicalSystem)


manager.scene.AddPointLight(chrono.ChVectorF(100, 100, 100),
                            chrono.ChColor(1, 1, 1), 500.0)




update_rate = 5.0          
horizontal_samples = 800   
vertical_samples = 300     
horizontal_fov = 2 * chrono.CH_C_PI    
max_vert_angle = chrono.CH_C_PI / 12   
min_vert_angle = -chrono.CH_C_PI / 6   
lag = 0.0                   
collection_time = 1.0 / update_rate
sample_radius = 2           
divergence_angle = 0.003    
return_mode = sens.LidarReturnMode_STRONGEST_RETURN


offset_pose = chrono.ChFrameD(
    chrono.ChVectorD(-8, 0, 1),
    chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))




lidar = sens.ChLidarSensor(
    mesh_body,              
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






lidar.PushFilter(sens.ChFilterLidarNoiseXYZI(0.01, 0.001, 0.001, 0.01))


lidar.PushFilter(sens.ChFilterVisualize(horizontal_samples, vertical_samples,
                                        "Raw Lidar Depth Data"))


lidar.PushFilter(sens.ChFilterDIAccess())
lidar.PushFilter(sens.ChFilterPCfromDepth())


lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1.0,
                                                  "Lidar Point Cloud"))


lidar.PushFilter(sens.ChFilterXYZIAccess())


lidar.PushFilter(sens.ChFilterSavePtCloud("output/lidar/"))


manager.AddSensor(lidar)




step_size = 1e-3
end_time = 40.0
ch_time = 0.0

orbit_radius = 8.0
orbit_rate = 0.5   

while ch_time < end_time:

    
    new_pose = chrono.ChFrameD(
        chrono.ChVectorD(
            orbit_radius * math.cos(ch_time * orbit_rate),
            orbit_radius * math.sin(ch_time * orbit_rate),
            1.0),
        chrono.Q_from_AngAxis(ch_time * orbit_rate + chrono.CH_C_PI,
                              chrono.ChVectorD(0, 0, 1)))
    lidar.SetOffsetPose(new_pose)

    
    manager.Update()

    
    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()
    if xyzi_buffer.HasData():
        xyzi_data = xyzi_buffer.GetXYZIData()
        print("Lidar buffer received:")
        print("  Width:  {}".format(xyzi_buffer.Width))
        print("  Height: {}".format(xyzi_buffer.Height))
        print("  First point [x,y,z,intensity]: {:.3f}, {:.3f}, {:.3f}, {:.3f}".format(
            xyzi_data[0, 0, 0],
            xyzi_data[0, 0, 1],
            xyzi_data[0, 0, 2],
            xyzi_data[0, 0, 3]))

    
    mphysicalSystem.DoStepDynamics(step_size)
    ch_time = mphysicalSystem.GetChTime()

print("Simulation complete.")