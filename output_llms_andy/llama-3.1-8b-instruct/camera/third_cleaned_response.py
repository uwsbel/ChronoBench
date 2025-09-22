import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    side = 2
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    box_body.SetBodyFixed(True)
    trimesh_shape = chrono.ChVisualShapeBox()
    trimesh_shape.SetBox(box_body)
    trimesh_shape.SetName("Box")
    box_body.AddVisualShape(trimesh_shape)

    
    trimesh_shape = chrono.ChVisualShapeBox()
    trimesh_shape.SetBox(box_body)
    trimesh_shape.SetName("Box Mesh")
    trimesh_shape.SetMutable(False)  

    
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))  
    mesh_body.AddVisualShape(trimesh_shape)  
    mesh_body.SetFixed(True)  
    mphysicalSystem.Add(mesh_body)  

    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))

    
    cam = sens.ChCameraSensor(
        mesh_body,              
        30,                     
        offset_pose,            
        1280,                  
        720,                   
        1.408                  
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(0)  
    cam.SetCollectionWindow(0)  

    
    
    
    
    if True:  
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif True:  
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif True:  
        
        pass

    
    if True:  
        cam.PushFilter(sens.ChFilterVisualize(1280, 720, "Before Grayscale Filter"))

    
    cam.PushFilter(sens.ChFilterRGBA8Access())

    
    if False:  
        cam.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/"))

    
    cam.PushFilter(sens.ChFilterGrayscale())

    
    if True:  
        cam.PushFilter(sens.ChFilterVisualize(640, 360, "Grayscale Image"))

    
    if False:  
        cam.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/gray/"))

    
    cam.PushFilter(sens.ChFilterImageResize(640, 360))

    
    cam.PushFilter(sens.ChFilterR8Access())

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10  
    orbit_rate = 0.5   
    ch_time = 0.0      

    t1 = time.time()  

    while ch_time < 20.0:
        
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(1e-3)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", 20.0, "Wall time:", time.time() - t1)






noise_model = True  


update_rate = 30


image_width = 1280
image_height = 720


fov = 1.408  


lag = 0


exposure_time = 0






step_size = 1e-3


end_time = 20.0


save = False


vis = True


out_dir = "SENSOR_OUTPUT/"


main()