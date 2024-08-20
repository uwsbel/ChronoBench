import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    box = chrono.ChBodyEasyBox(side=1000, side=1000, side=1000, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))  
    box.SetFixed(True)  

    
    box_shape = chrono.ChVisualShapeBox()
    box_shape.SetMesh(box)
    box_shape.SetName("Box")
    box_shape.SetMutable(False)  

    
    camera = sens.ChCameraSensor(
        box,              
        update_rate,      
        offset_pose,      
        image_width,      
        image_height,     
        fov                 
    )
    camera.SetName("Camera Sensor")
    camera.SetLag(lag)  
    camera.SetCollectionWindow(exposure_time)  

    
    
    
    
    if noise_model == "CONST_NORMAL":
        camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif noise_model == "PIXEL_DEPENDENT":
        camera.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif noise_model == "NONE":
        
        pass

    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Before Grayscale Filter"))

    
    camera.PushFilter(sens.ChFilterRGBA8Access())

    
    if save:
        camera.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    
    camera.PushFilter(sens.ChFilterGrayscale())

    
    if vis:
        camera.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    
    if save:
        camera.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    
    camera.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))

    
    camera.PushFilter(sens.ChFilterR8Access())

    
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 10  
    orbit_rate = 0.5   
    ch_time = 0.0      

    t1 = time.time()  

    while ch_time < end_time:
        
        camera.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        
        rgba8_buffer = camera.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print('RGBA8 buffer received from cam. Camera resolution: {0}x{1}'.format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))

        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time() - t1)






noise_model = "CONST_NORMAL"  


lens_model = sens.PINHOLE  


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