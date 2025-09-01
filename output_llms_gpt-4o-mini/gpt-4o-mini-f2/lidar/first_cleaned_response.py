import pychrono as ch
import pychrono.sensor as sens
import math


system = ch.ChSystemNSC()
system.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)


mesh = ch.ChBodyEasyMesh(
    "../data/meshes/tri_mesh.obj",  
    1000,  
    True,  
    True,  
    ch.ChMaterialNSC()  
)
mesh.SetPos(ch.ChVector3d(0, 0, 0))  
mesh.SetFixed(True)  
system.Add(mesh)  


manager = sens.ChSensorManager(system)


lidar = sens.ChLidarSensor(mesh,  
                           ch.ChFramed(ch.ChVector3d(0.0, 0, 1.7), ch.QuatFromAngleAxis(0, ch.ChVector3d(0, 1, 0))),  
                           0.1,  
                           64,  
                           64,  
                           1.0,  
                           ch.SID(1.0, 1.0, 1.0),  
                           100.0,  
                           sens.LidarBeamShape_RECTANGULAR,  
                           0.004,  
                           0.002,  
                           sens.LidarReturnMode_STRONGEST_RETURN  
)
lidar.SetName("Lidar Sensor")
lidar.PushFilter(sens.ChFilterDIAccess())  
lidar.PushFilter(sens.ChFilterPCfromDepth())  
lidar.PushFilter(sens.ChFilterVisualizePointCloud(800, 600, 1.0, "Lidar PC data"))  
lidar.PushFilter(sens.ChFilterSaverPointCloud("../data/lidar_pc/"))  
manager.AddSensor(lidar)  


orbit_radius = 5  
orbit_speed = 0.5  
offset = 0  


for t in range(0, 1000):
    time = t * 0.02  
    
    lidar.SetPos(ch.ChVector3d(orbit_radius * math.cos(orbit_speed * time + offset), 
                               orbit_radius * math.sin(orbit_speed * time + offset), 
                               1.7))
    
    print(lidar.GetCurrentBuffer())
    
    system.DoStepDynamics(0.02)