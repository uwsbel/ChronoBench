import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import os 

def main():
    
    
    
    
    noise_model = "CONST_NORMAL"  

    
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
    if save:
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
        if not os.path.exists(out_dir + "rgb/"):
            os.makedirs(out_dir + "rgb/")
        if not os.path.exists(out_dir + "gray/"):
            os.makedirs(out_dir + "gray/")


    
    side = 1.0 

    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    
    box_body = chrono.ChBodyEasyBox(side / 2, side / 2, side / 2,  
                                     1000,                          
                                     True,                          
                                     True)                          
    
    
    box_body.SetPos(chrono.ChVector3d(0, 0, side / 2))
    box_body.SetFixed(True)  
    mphysicalSystem.Add(box_body)  

    
    
    
    if len(box_body.GetVisualShapes()) > 0:
        visual_shape = box_body.GetVisualShape(0) 
        if visual_shape:
            
            if visual_shape.material_list and len(visual_shape.material_list) > 0:
                
                visual_shape.material_list[0].SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
            else:
                
                custom_material = chrono.ChVisualMaterial()
                custom_material.SetKdTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
                visual_shape.material_list.append(custom_material)

    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, 
                               chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0)) 

    
    
    
    
    
    
    original_rotation_quat = chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0))
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), original_rotation_quat)

    
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

    
    
    
    
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif noise_model == "NONE":
        
        pass

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale Filter"))

    
    cam.PushFilter(sens.ChFilterRGBA8Access())

    
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

    t1 = time.time()  

    while ch_time < end_time:
        
        
        
        
        cam_orbit_pos = chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                                          -orbit_radius * math.sin(ch_time * orbit_rate),
                                          1) 
        cam_orbit_rot = chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        
        cam.SetOffsetPose(chrono.ChFramed(cam_orbit_pos, cam_orbit_rot))

        
        rgba8_buffer_ptr = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer_ptr.HasData():
            
            pixel_data_accessor = rgba8_buffer_ptr.GetRGBA8Data() 
            print(f'RGBA8 buffer received from cam. Camera resolution: {rgba8_buffer_ptr.Width}x{rgba8_buffer_ptr.Height}')
            
            if rgba8_buffer_ptr.Width > 0 and rgba8_buffer_ptr.Height > 0:
                first_pixel_rgba = pixel_data_accessor[0,0] 
                print(f'First Pixel (RGBA): {first_pixel_rgba}')

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)


if __name__ == '__main__':
    main()