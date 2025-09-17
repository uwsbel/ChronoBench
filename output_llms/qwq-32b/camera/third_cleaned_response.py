import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 1.0  
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)  
    box_body.SetPos(chrono.ChVectorD(0, 0, 0))
    box_body.SetBodyFixed(True)
    mphysicalSystem.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVectorF(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVectorF(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVectorF(1, 0, 0), chrono.ChVectorF(0, -1, 0))

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVectorD(-7, 0, 3),
        chrono.Q_from_AngAxis(2, chrono.ChVectorD(0, 1, 0))
    )

    cam = sens.ChCameraSensor(
        box_body,              
        offset_pose,           
        update_rate,           
        image_width,           
        image_height,          
        fov                   
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetExposureTime(exposure_time)  

    
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
        cam.PushFilter(sens.ChFilterVisualize(int(image_width/2), int(image_height/2), "Grayscale Image"))
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))
    
    cam.PushFilter(sens.ChFilterImageResize(int(image_width/2), int(image_height/2)))
    cam.PushFilter(sens.ChFilterR8Access())
    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.5
    ch_time = 0.0
    t1 = time.time()

    while ch_time < end_time:
        
        angle = ch_time * orbit_rate
        pos = chrono.ChVectorD(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1
        )
        rot = chrono.Q_from_AngAxis(angle, chrono.ChVectorD(0, 0, 1))
        cam.SetOffsetPose(chrono.ChFramed(pos, rot))
        
        
        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(f'RGBA8 buffer received from cam. Resolution: {rgba8_buffer.Width}x{rgba8_buffer.Height}')
            print(f'First Pixel: {rgba8_data[0, 0, :]}')
        
        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

    print(f"Sim time: {end_time}, Wall time: {time.time() - t1}")




side = 1.0
step_size = 1e-3
end_time = 20.0
save = False
vis = True
out_dir = "SENSOR_OUTPUT/"




noise_model = "CONST_NORMAL"
update_rate = 30
image_width = 1280
image_height = 720
fov = 1.408
lag = 0
exposure_time = 0

main()