import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time

def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    side = 1.0

    
    box_shape = chrono.ChVisualShapeBox(side, side, side)
    box_shape.SetName("Box Shape")
    box_shape.SetMutable(False)  

    
    box_body = chrono.ChBodyEasyBox(side, side, side, 1000)
    box_body.SetPos(chrono.ChVector3d(0, 0, 0))  
    box_body.AddVisualShape(box_shape)  
    box_body.SetFixed(True)  
    mphysicalSystem.Add(box_body)  

    
    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(9, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(16, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(chrono.ChVector3f(23, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 3), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    
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
        cam.PushFilter(sens.ChFilterSave
print("error happened with only start ```python")