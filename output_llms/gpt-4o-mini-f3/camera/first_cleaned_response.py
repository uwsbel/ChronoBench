import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    
    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-4, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
    )
    cam = sens.ChCameraSensor(
        mesh_body,              
        update_rate,            
        offset_pose,            
        image_width,            
        image_height,           
        fov                    
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    
    
    
    if noise_model == " CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == " PIXEL_SPECIAL":
        cam.PushFilter(sens.ChFilterCameraNoisePixSel(0.02, 0.03))
    elif noise_model == " BATEMAN":
        cam.PushFilter(sens.ChFilterCameraNoiseBateman(0.02, 0.03))

    if absense_percentage > 0:
        cam.PushFilter(sens.ChFilterVisualize(absense_percentage, 255, 0, 0))

    if visualize:
        
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    
    if save_input:
        cam.PushFilter(sens.ChFilterSave(out_dir + "input/"))

    
    cam.PushFilter(sens.ChFilterGrayscale())
    if visualize:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    
    if save_gray:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    
    cam.PushFilter(sens.ChFilterImageFFT())

    
    if motion_detect:
        cam.PushFilter(sens.ChFilterMotionDetect(10, 2, 2, 2))

    
    if visualize:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Edge Detection"))

    
    
    
    pc = sens.ChPointCloudSensor(
        mesh_body,              
        update_rate,            
        offset_pose,            
        pc_width,               
        pc_height,              
        pc_fov,                 
        noiseless_sample_radius,  
        sample_radius,          
        sample_theta,           
        (0, 0, 255),            
        chisetrange,            
        stingray,               
        10                      
    )
    pc.SetName("Point Cloud Sensor")
    pc.SetLag(lag)
    pc.SetCollectionWindow(exposure_time)
    if visualize:
        pc.PushFilter(sens.ChFilterVisualize(int(pc_width), int(pc_height), "Point Cloud"))

    
    if save_ptcloud:
        pc.PushFilter(sens.ChFilterSAP(out_dir + "pc/"))

    
    manager.AddSensor(cam)
    manager.AddSensor(pc)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.5
    ch_time = 0.0

    t1 = time.time()
    while True:
        
        cambody_pos = mesh_body.GetPos()
        cambody_rot = mesh_body.GetRot()
        cam.SetOffsetPose(
            chrono.ChFramed(
                cambody_pos + chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
                cambody_rot
            )
        )

        
        manager.Update()

        
        ch_time += time_step

        
        if ch_time > 10:
            break

    print("Sim time:", time.time() - t1)






noise_model = "CONST_NORMAL"  


lens_model = "RECTILINEAR"  


update_rate = 30


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0






pc_width = 640
pc_height = 480


pc_fov = 1.728


noise_model = "GAUSSIAN"


sample_radius = 2


noiseless_sample_radius = 1






time_step = 1e-3


ch_time = 0.0


save_camera = False
save_ptcloud = False
render = True
motion_detect = False


out_dir = "SENSOR_OUTPUT/"


main()