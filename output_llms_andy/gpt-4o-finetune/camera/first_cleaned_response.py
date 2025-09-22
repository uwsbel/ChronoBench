import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as sens


sys = chrono.ChSystemNSC()


mesh_scale = chrono.ChVector3d(3, 3, 3)  
triang_mesh = chrono.ChTriangleMeshConnected()

triang_mesh.LoadWavefrontMesh(
    chrono.GetChronoDataFile("gripper/wrist_triangles.obj"), False, True)

triang_mesh.Transform(
    mesh_scale, chrono.ChVector3d(-1.8, 0, -1.5))


mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(triang_mesh)
mesh_shape.SetName("Wrist")
mesh_shape.SetMutable(False)


body = chrono.ChBody()
body.AddVisualShape(mesh_shape)
body.SetFixed(True)  
sys.AddBody(body)  


manager = sens.ChSensorManager(sys)


offset_pose = chrono.ChFramed(
    chrono.ChVector3d(0, 1, 0), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera = sens.ChCameraSensor(
    body,              
    30,                
    offset_pose,       
    1280, 720,          
    1.5708,            
    1,                 
    sens.IMAGE_PIXELS, 
    3.8                 
)
camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Frustum buffer"))
camera.PushFilter(sens.ChFilterRGBA8ToIntensity())
camera.PushFilter(sens.ChFilterObservationToTexture())
manager.AddSensor(camera)  


offset_pose2 = chrono.ChFramed(
    chrono.ChVector3d(0, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
camera2 = sens.ChCameraSensor(
    body,              
    30,                
    offset_pose2,      
    1280, 720,          
    1.5708,            
    1,                 
    sens.IMAGE_INTENSITY, 
    3.8                 
)
camera2.PushFilter(sens.ChFilterVisualize(1280, 720, "Gray buffer"))
camera2.PushFilter(sens.ChFilterDenoising())
camera2.PushFilter(sens.ChFilterMean(5))
camera2.PushFilter(sens.ChFilterR recentToIntensity())
manager.AddSensor(camera2)  


step_size = 1e-3
run_time = 100


sys.Setup()


rot = 0
counter = 0
while counter < 2 * run_time:
    
    cam_offset = chrono.ChFramed(chrono.ChVector3d(3 * math.cos(rot), 2, 3 * math.sin(rot)), chrono.QuatFromAngleAxis(rot, chrono.ChVector3d(0, 1, 0)))
    camera.SetOffsetPose(cam_offset)
    
    
    print("Image buffer out ", counter, " ", camera.GetMostRecentIntensityBuffer())
    
    
    manager.Update()
    
    
    sys.DoStepDynamics(step_size)
    rot += step_size / 2  
    counter += 1