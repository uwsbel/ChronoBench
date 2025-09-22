import math
import os


import pychrono.core as chrono
import pychrono.sensor as sens




chrono.SetChronoDataPath(os.getenv("CHRONO_DATA_DIR", ""))  
system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))




mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)


mesh = chrono.geometry.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj",                       
                       False,                            
                       True)                             

mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_shape.SetName("tri_mesh")
mesh_shape.SetStatic(True)
mesh_body.AddAsset(mesh_shape)

system.Add(mesh_body)





manager = sens.ChSensorManager(system)
manager.SetVerbose(True)
manager.SetCameraListIndex(0)


update_rate = 20.0            
horiz_samples = 360           
vert_samples  =  30           
horiz_FOV_min = -chrono.CH_C_PI
horiz_FOV_max =  chrono.CH_C_PI
vert_FOV_min  = -chrono.CH_C_PI/6
vert_FOV_max  =  chrono.CH_C_PI/6

lidar = sens.ChLidar(parent=mesh_body,
                     update_rate=update_rate,
                     horizontal_samples=horiz_samples,
                     vertical_samples=vert_samples,
                     horiz_min=horiz_FOV_min,
                     horiz_max=horiz_FOV_max,
                     vert_min=vert_FOV_min,
                     vert_max=vert_FOV_max)

lidar.SetName("OrbitingLiDAR")
lidar.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))  
lidar.SetMinMaxRange(0.2, 20.0)                                  


noise = sens.ChFilterGaussianNoise(0.0, 0.01)  
lidar.PushFilter(noise)


vis = sens.ChFilterVisualizePointCloud(chrono.ChColor(0.0, 1.0, 0.0), 3)
lidar.PushFilter(vis)



save = sens.ChFilterSaveXYZ("lidar_output", sens.ChFilterSaveXYZ.SEPARATE_FILES)
lidar.PushFilter(save)


manager.AddSensor(lidar)





dt_sim = 1e-3
total_time = 10.0
n_steps = int(total_time / dt_sim)
orbit_radius = 5.0
orbit_height = 2.0
orbit_speed  = 0.5     

for step in range(n_steps):
    t = system.GetChTime()
    
    
    theta = orbit_speed * t
    x = orbit_radius * math.cos(theta)
    y = orbit_radius * math.sin(theta)
    z = orbit_height
    
    
    target = chrono.ChVectorD(0, 0, 0)
    pos    = chrono.ChVectorD(x, y, z)
    forward = (target - pos).GetNormalized()
    
    up = chrono.VECT_Z
    
    
    m = chrono.ChMatrix33D()
    m.Set_A_Xdir(forward, up)
    rot = chrono.Q_from_AngX(m)  
    
    pose = chrono.ChFrameD(pos, rot)
    lidar.SetOffsetPose(pose)
    
    
    manager.Update()            
    system.DoStepDynamics(dt_sim)
    
    
    buffer = lidar.GetBufferXYZ()
    npts = 0 if buffer is None else len(buffer)
    print(f"[{t:6.3f} s] LiDAR points: {npts}")