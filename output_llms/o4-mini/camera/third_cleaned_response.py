import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    system = chrono.ChSystemNSC()

    
    
    
    side = 2.0  
    density = 1000.0
    
    box_body = chrono.ChBodyEasyBox(side, side, side, density, True, True)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))
    
    tex = chrono.ChTexture()
    tex.SetTextureFilename(chrono.GetChronoDataFile("sensor/textures/loader.png"))
    box_body.GetAssets().push_back(tex)
    system.Add(box_body)

    
    
    
    manager = sens.ChSensorManager(system)

    
    intensity = 1.0
    manager.scene.AddPointLight(
        chrono.ChVector3f(2, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0)
    manager.scene.AddPointLight(
        chrono.ChVector3f(9, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0)
    manager.scene.AddPointLight(
        chrono.ChVector3f(16, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0)
    manager.scene.AddPointLight(
        chrono.ChVector3f(23, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0)
    manager.scene.AddAreaLight(
        chrono.ChVector3f(0, 0, 4),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
        chrono.ChVector3f(1, 0, 0),
        chrono.ChVector3f(0, -1, 0),
    )

    
    
    
    
    offset = chrono.ChFrameD(
        chrono.ChVector3d(-7, 0, 3),
        chrono.Q_from_AngAxis(2.0, chrono.ChVector3d(0, 1, 0)),
    )

    cam = sens.ChCameraSensor(
        box_body,       
        update_rate,    
        offset,         
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
    

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Before Grayscale"))

    
    cam.PushFilter(sens.ChFilterRGBA8Access())
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    
    cam.PushFilter(sens.ChFilterGrayscale())
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width//2, image_height//2, "Gray Image"))
    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    
    cam.PushFilter(sens.ChFilterImageResize(image_width//2, image_height//2))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10.0
    orbit_rate = 0.5
    ch_time = 0.0
    t_start = time.time()

    while ch_time < end_time:
        
        angle = ch_time * orbit_rate
        new_pos = chrono.ChVector3d(
            -orbit_radius * math.cos(angle),
            -orbit_radius * math.sin(angle),
            1.0
        )
        new_rot = chrono.Q_from_AngAxis(angle, chrono.ChVector3d(0, 0, 1))
        cam.SetOffsetPose(chrono.ChFrameD(new_pos, new_rot))

        
        buf = cam.GetMostRecentRGBA8Buffer()
        if buf.HasData():
            data = buf.GetRGBA8Data()
            print(f"Frame @ t={ch_time:.3f}s  Resolution: {buf.Width}×{buf.Height}")
            print("  First pixel RGBA:", data[0, 0, :])

        
        manager.Update()
        system.DoStepDynamics(step_size)
        ch_time = system.GetChTime()

    print(f"Simulated {end_time}s of Chrono time in {time.time() - t_start:.2f}s wall time.")





noise_model = "CONST_NORMAL"  

update_rate   = 30        
image_width   = 1280      
image_height  = 720       
fov           = 1.408     
lag           = 0.0       
exposure_time = 0.0       




step_size = 1e-3  
end_time  = 20.0  


save   = False
vis    = True
out_dir = "SENSOR_OUTPUT/"

if __name__ == "__main__":
    main()