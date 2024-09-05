import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    side = 2  
    box = chrono.ChBodyEasyBox(side, side, side, 1000)
    box.SetPos(chrono.ChVector3d(0, 0, 0))  
    box.SetName("Box")
    box.SetFixed(True)  
    mphysicalSystem.Add(box)  

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    cam = sens.ChCameraSensor(
        box,              
        update_rate,            
        offset_pose,            
        image_width,            
        image_height,           
        fov                     
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(lag)
    cam.SetCollectionWindow(exposure_time)

    

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.5
    ch_time = 0.0

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