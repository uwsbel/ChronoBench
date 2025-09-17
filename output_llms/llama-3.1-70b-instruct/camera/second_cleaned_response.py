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

    
    intensity = 1.0  
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2.5, 100), chrono.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddAreaLight(chrono.ChVector3f(0, 0, 4), chrono.ChColor(intensity, intensity, intensity), 500.0, chrono.ChVector3f(1, 0, 0), chrono.ChVector3f(0, -1, 0))

    
    
    
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-7, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))

    
    cam = sens.ChCameraSensor(
        mesh_body,              
        30,                     
        offset_pose,            
        960,                    
        480,                    
        1.408                   
    )
    cam.SetName("Camera Sensor")
    cam.SetLag(0)  
    cam.SetCollectionWindow(0)  

    
    
    
    
    if "CONST_NORMAL":
        cam.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))  
    elif "PIXEL_DEPENDENT":
        cam.PushFilter(sens.ChFilterCameraNoisePixDep(0.02, 0.03))  
    elif "NONE":
        
        pass

    
    if True:
        cam.PushFilter(sens.ChFilterVisualize(960, 480, "Before Grayscale Filter"))

    
    cam.PushFilter(sens.ChFilterRGBA8Access())

    
    if True:
        cam.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/" + "rgb/"))

    
    cam.PushFilter(sens.ChFilterGrayscale())

    
    if True:
        cam.PushFilter(sens.ChFilterVisualize(int(960 / 2), int(480 / 2), "Grayscale Image"))

    
    if True:
        cam.PushFilter(sens.ChFilterSave("SENSOR_OUTPUT/" + "gray/"))

    
    cam.PushFilter(sens.ChFilterImageResize(int(960 / 2), int(480 / 2)))

    
    cam.PushFilter(sens.ChFilterR8Access())

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10  
    orbit_rate = 0.1   
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


main()