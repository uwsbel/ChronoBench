import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    side = 1.0
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))  
    box_body.SetBodyFixed(True)  
    mphysicalSystem.Add(box_body)  

    
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetBoxDimensions(chrono.ChVector3d(side, side, side))
    box_shape.SetName("Box")
    box_shape.SetColor(chrono.ChColor(1, 0, 0))  
    box_body.AddVisualShape(box_shape)  

    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    
    update_rate = 30  
    image_width = 1280  
    image_height = 720  
    fov = 1.408  
    lag = 0  
    exposure_time = 0  
    cam = sens.ChCameraSensor(
        box_body,              
        update_rate,            
        offset_pose,            
        image_width,            
        image_height,           
        fov                     
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)  
    cam.SetCollectionWindow(exposure_time)  

    
    
    
    
    noise_model = "CONST_NORMAL"  
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif noise_model == "NONE":
        
        pass

    
    vis = True
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    
    cam.PushFilter(sens.ChFilterRGBA8Access())

    
    save = False
    out_dir = "SENSOR_OUTPUT/"
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    
    cam.PushFilter(sens.ChFilterGrayscale())

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    
    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    
    cam.PushFilter(sens.ChFilterR8Access())

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10  
    orbit_rate = 0.5   
    ch_time = 0.0      
    step_size = 1e-3  
    end_time = 20.0  

    t1 = time.time()  

    while ch_time < end_time:
        
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


main()