import math
import os
import numpy as np
import pychrono.core as chrono
import pychrono.sensor as sens




chrono.SetChronoDataPath(chrono.GetChronoDataPath())          
system       = chrono.ChSystemNSC()                           
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))                   




OBJ_FILE = os.path.join(os.getcwd(), "my_mesh.obj")           

mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(OBJ_FILE, True, True)


mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_shape.SetBackfaceCull(True)


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.AddVisualShape(mesh_shape)
system.Add(mesh_body)




mgr = sens.ChSensorManager(system)


mgr.scene.AddPointLight(chrono.ChVectorF(5, 5, 5), chrono.ChVectorF(1, 1, 1), 500)




update_rate        = 10.0          
h_samples          = 512           
v_beams            =  16           
h_fov              = 2.0 * math.pi 
max_vert_angle     =  math.radians(10)
min_vert_angle     = -math.radians(10)
lag                = 0.0
collection_window  = 1.0 / update_rate


init_pose = chrono.ChFrameD(chrono.ChVectorD(2, 0, 0.5), chrono.ChQuaternionD(1, 0, 0, 0))

lidar = sens.ChLidarSensor(mesh_body,              
                           update_rate,
                           init_pose,
                           h_samples,
                           v_beams,
                           h_fov,
                           max_vert_angle,
                           min_vert_angle,
                           lag,
                           collection_window)


lidar.AddFilter(sens.ChFilterLidarNoiseXYZIB())                     
lidar.AddFilter(sens.ChFilterVisualizeXYZIB(640, 480, "Lidar"))     
lidar.AddFilter(sens.ChFilterSaveXYZIB(os.path.join(os.getcwd(),
                                                    "lidar_output")))   

mgr.AddSensor(lidar)




orbit_radius   = 2.0      
orbit_height   = 0.5      
orbit_period   = 5.0      
def place_lidar(t):
    
    angle = 2.0 * math.pi * (t % orbit_period) / orbit_period
    x =  orbit_radius * math.cos(angle)
    y =  orbit_radius * math.sin(angle)
    z =  orbit_height

    
    pos      = chrono.ChVectorD(x, y, z)
    dir_vec  = chrono.ChVectorD(0, 0, 0) - pos
    dir_vec.Normalize()
    up       = chrono.ChVectorD(0, 0, 1)
    rot      = chrono.Q_from_TwoVectors(chrono.ChVectorD(0, 0, 1), dir_vec)  

    lidar.SetOffsetPose(chrono.ChFrameD(pos, rot))




step_size   = 1.0e-3
end_time    = 10.0                               
realtime    = chrono.ChRealtimeStepTimer()

print("Running simulation ...")
while system.GetChTime() < end_time:

    
    place_lidar(system.GetChTime())

    
    mgr.Update()

    
    system.DoStepDynamics(step_size)

    
    realtime.Spin(step_size)

    
    buff = lidar.GetMostRecentBuffer()
    if buff is not None and buff.HasData():
        
        xyzib = np.array(buff.GetXYZIB()).reshape(-1, 5)   
        print(f"[t = {system.GetChTime():6.3f}] "
              f"{xyzib.shape[0]} points captured")

print("Simulation complete.")