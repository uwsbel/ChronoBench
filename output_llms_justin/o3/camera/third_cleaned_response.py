import math
import os
import time

import pychrono.core as chrono
import pychrono.sensor as sens






noise_model   = "CONST_NORMAL"


update_rate   = 30.0                                
image_width   = 1280                                
image_height  = 720                                 
fov           = 1.408                               
lag           = 0.0                                 
exposure_time = 0.0                                 


step_size     = 1e-3                                
end_time      = 20.0                                


save          = False                               
vis           = True                                
out_dir       = "SENSOR_OUTPUT/"                    


side          = 2.0                                 


def main():
    
    
    
    system = chrono.ChSystemNSC()

    
    
    
    box = chrono.ChBodyEasyBox(side, side, side,         
                               1000.0,                   
                               True, True)               

    box.SetPos(chrono.ChVectorD(0.0, 0.0, side / 2.0))   
    box.SetBodyFixed(True)

    
    tex_file = chrono.GetChronoDataFile("textures/white.png")
    for i in range(box.GetNumVisualShapes()):
        box.GetVisualShape(i).SetTexture(tex_file)

    system.Add(box)

    
    
    
    manager = sens.ChSensorManager(system)

    light_int = 1.0
    manager.scene.AddPointLight(chrono.ChVectorF(2,   2.5, 4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0)
    manager.scene.AddPointLight(chrono.ChVectorF(-2,  2.5, 4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0)
    manager.scene.AddAreaLight (chrono.ChVectorF(0,   0,   4),
                                chrono.ChColor(light_int, light_int, light_int), 500.0,
                                chrono.ChVectorF(1, 0, 0), chrono.ChVectorF(0, -1, 0))

    
    
    
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-7, 0, 3),
                                  chrono.Q_from_AngAxis(2.0, chrono.ChVectorD(0, 1, 0)))

    cam = sens.ChCameraSensor(box, update_rate, offset_pose,
                              image_width, image_height, fov)

    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    
    if noise_model == "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))
    elif noise_model == "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height,
                                              "RGB Image (pre-gray)"))

    cam.PushFilter(sens.ChFilterRGBA8Access())
    if save:
        cam.PushFilter(sens.ChFilterSave(os.path.join(out_dir, "rgb/")))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width // 2, image_height // 2,
                                              "Grayscale Image"))
    if save:
        cam.PushFilter(sens.ChFilterSave(os.path.join(out_dir, "gray/")))

    cam.PushFilter(sens.ChFilterImageResize(image_width // 2, image_height // 2))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    
    if save:
        os.makedirs(os.path.join(out_dir, "rgb/"),  exist_ok=True)
        os.makedirs(os.path.join(out_dir, "gray/"), exist_ok=True)

    
    
    
    orbit_radius = 10.0     
    orbit_rate   = 0.5      

    ch_time   = 0.0
    wall_t0   = time.time()

    while ch_time < end_time:
        
        cam.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate),
                                 -orbit_radius * math.sin(ch_time * orbit_rate),
                                 3.0),
                chrono.Q_from_AngAxis(ch_time * orbit_rate, chrono.ChVectorD(0, 0, 1))
            )
        )

        
        rgba_buf = cam.GetMostRecentRGBA8Buffer()
        if rgba_buf.HasData():
            data = rgba_buf.GetRGBA8Data()
            print(f"t = {ch_time:6.3f}  |  Image {rgba_buf.Width}x{rgba_buf.Height}  "
                  f"|  first pixel RGBA = {data[0, 0, :]}")

        manager.Update()                     
        system.DoStepDynamics(step_size)     
        ch_time = system.GetChTime()

    print(f"Simulation time {end_time} s  |  Wall time {time.time() - wall_t0:.2f} s")





if __name__ == "__main__":
    main()