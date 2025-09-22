import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math
import time


chrono.SetChronoDataPath(chrono.GetChronoDataPath())
system = chrono.ChSystemSMC()




mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))


mesh_shape = chrono.ChTriangleMeshConnected()
mesh_shape.LoadWavefrontMesh("my_mesh.obj")  
mesh_shape.RepairDuplicateVertexes(1e-9)

visual_shape = chrono.ChTriangleMeshShape()
visual_shape.SetMesh(mesh_shape)
visual_shape.SetName("mesh")
visual_shape.SetColor(chrono.ChColor(0.4, 0.4, 0.8))

mesh_body.AddAsset(visual_shape)
system.Add(mesh_body)


manager = sens.ChSensorManager(system)


update_rate = 30.0  
exposure_time = 1.0 / update_rate


cam_width = 640
cam_height = 480
fov = 1.4  


orbit_radius = 2.0
orbit_height = 0.5
orbit_speed = 0.5  


def get_camera_pose(theta):
    x = orbit_radius * math.cos(theta)
    y = orbit_height
    z = orbit_radius * math.sin(theta)
    pos = chrono.ChVectorD(x, y, z)
    
    target = chrono.ChVectorD(0, 0, 0)
    up = chrono.ChVectorD(0, 1, 0)
    
    dir = (target - pos).GetNormalized()
    right = up.Cross(dir).GetNormalized()
    up2 = dir.Cross(right)
    rot = chrono.ChMatrix33D()
    rot.Set_A_axis(right, up2, dir)
    return chrono.ChFrameD(pos, rot)


theta = 0.0
cam_pose = get_camera_pose(theta)


camera = sens.ChCameraSensor(
    mesh_body,                
    update_rate,              
    cam_pose,                 
    cam_width, cam_height,    
    fov                       
)


camera.PushFilter(sens.ChFilterCameraNoise(sens.CameraNoiseModel_TYPE_GAUSSIAN, 0.0, 0.03))


camera.PushFilter(sens.ChFilterVisualize(cam_width, cam_height, "Camera View"))


camera.PushFilter(sens.ChFilterRGBA8Access())


manager.AddSensor(camera)



step_size = 1.0 / 240.0  
end_time = 5.0           
steps = int(end_time / step_size)

print("Starting simulation...")

for i in range(steps):
    time_curr = i * step_size

    
    theta = orbit_speed * time_curr
    cam_pose = get_camera_pose(theta)
    camera.SetOffsetPose(cam_pose)

    
    system.DoStepDynamics(step_size)
    manager.Update()

    
    img = camera.GetMostRecentRGBA8Buffer()
    if img is not None:
        
        print(f"Step {i}, Time {time_curr:.3f}s, Camera buffer shape: {img.shape}, Mean pixel value: {img.mean():.2f}")
    else:
        print(f"Step {i}, Time {time_curr:.3f}s, No camera data yet.")

    
    

print("Simulation complete.")