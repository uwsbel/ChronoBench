import math
import pychrono as chrono
import pychrono.sensor as sens            



chrono.SetChronoDataPath(chrono.GetChronoDataPath())        
system        = chrono.ChSystemSMC()                         
time_step     = 1.0/240.0                                    
end_time      = 5.0                                          



mesh          = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("my_mesh.obj", save_vertex_normals=True, save_uv=False)

mesh_shape    = chrono.ChVisualShapeTriangleMesh(mesh)
mesh_shape.SetBackfaceCull(True)

mesh_body     = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.AddVisualShape(mesh_shape)

system.Add(mesh_body)



manager       = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorF(2,3,2),  chrono.ChColor(1,1,1), 10)
manager.scene.AddPointLight(chrono.ChVectorF(-2,3,-2),chrono.ChColor(1,1,1), 10)



cam_rate      = 30.0                                         
res_x, res_y  = 640, 480
fov           = 60                                           
initial_frame = chrono.ChFrameD(chrono.ChVectorD(3,1,0),     
                                chrono.ChQuaternionD(1,0,0,0))

camera = sens.ChCameraSensor(mesh_body,                      
                             cam_rate,                       
                             initial_frame,                  
                             res_x, res_y,                   
                             math.radians(fov))              



access_rgba    = sens.ChFilterRGBA8Access()
camera.PushFilter(access_rgba)


camera.PushFilter(sens.ChFilterCameraNoiseNormal(0,          
                                                 0.02,0.02,0.02)) 


camera.PushFilter(sens.ChFilterVisualize(res_x, res_y, "Orbiting camera"))


manager.AddSensor(camera)



def build_lookat_quat(from_pos, to=chrono.ChVectorD(0,0,0)):
    
    dir_z  = (to - from_pos)
    if dir_z.Length() < 1e-12:
        return chrono.ChQuaternionD(1,0,0,0)
    dir_z.Normalize()

    up     = chrono.ChVectorD(0,1,0)
    dir_x  = up.Cross(dir_z);  dir_x.Normalize()
    dir_y  = dir_z.Cross(dir_x)

    M      = chrono.ChMatrix33D()
    M.Set_A_Xdir(dir_x, dir_y, dir_z)      
    return chrono.ChQuaternionD(M)



omega         = 2*math.pi/5.0                               
step          = 0
print("Running simulation …")
while system.GetChTime() < end_time:
    
    
    t          = system.GetChTime()
    radius     = 3.0
    cam_pos    = chrono.ChVectorD(radius*math.cos(omega*t), 1.0,
                                  radius*math.sin(omega*t))
    cam_rot    = build_lookat_quat(cam_pos)                 
    camera.SetOffsetPose(chrono.ChFrameD(cam_pos, cam_rot))

    
    
    system.DoStepDynamics(time_step)
    manager.Update()                                        
    manager.PostProcess()                                   

    
    
    if access_rgba.BufferIsReady():                         
        img = access_rgba.GetBuffer()                       
        
        r,g,b,a = img[0,0]
        print(f"[{step:04}]  t = {t:6.3f}  first-px = ({r:3},{g:3},{b:3},{a:3})")

    step += 1

print("Simulation finished.")