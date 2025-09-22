import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time
import os  






noise_model = "CONST_NORMAL"  


update_rate = 30


image_width = 960   
image_height = 480  


fov = 1.408  


lag = 0


exposure_time = 0








step_size = 1e-3


end_time = 20.0


save = True  


vis = True


out_dir = "SENSOR_OUTPUT/"

def main():
    
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    
    mmesh = chrono.ChTriangleMeshConnected()
    try:
        mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    except Exception as e:
        print(f"Error: Could not load HMMWV mesh.")
        print(f"Please ensure CHRONO_DATA_DIR is set correctly and the file 'vehicle/hmmwv/hmmwv_chassis.obj' exists.")
        print(f"Details: {e}")
        return  

    
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
        
        rgb_save_path = os.path.join(out_dir, "rgb")
        gray_save_path = os.path.join(out_dir, "gray")
        os.makedirs(rgb_save_path, exist_ok=True)
        os.makedirs(gray_save_path, exist_ok=True)
        
        
        cam.PushFilter(sens.ChFilterSave(rgb_save_path + os.sep))


    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    if save:
        gray_save_path = os.path.join(out_dir, "gray") 
        
        cam.PushFilter(sens.ChFilterSave(gray_save_path + os.sep))


    cam.PushFilter(sens.ChFilterImageResize(int(image_width / 2), int(image_height / 2)))
    cam.PushFilter(sens.ChFilterR8Access())
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10
    orbit_rate = 0.1  
    ch_time = 0.0

    t1 = time.time()

    print(f"Simulation running for {end_time} seconds...")
    if save:
        print(f"Saving images to '{os.path.abspath(out_dir)}rgb/' and '{os.path.abspath(out_dir)}gray/'")
    if vis:
        print("Visualization windows will be displayed if platform supports it.")

    frame_count = 0
    while ch_time < end_time:
        cam_orbit_pos = chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate),
                                          -orbit_radius * math.sin(ch_time * orbit_rate),
                                          1) 
        cam_orbit_rot = chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))
        
        cam.SetOffsetPose(chrono.ChFramed(cam_orbit_pos, cam_orbit_rot))

        manager.Update()
        mphysicalSystem.DoStepDynamics(step_size)
        ch_time = mphysicalSystem.GetChTime()

        
        if frame_count % update_rate == 0 : 
            rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
            if rgba8_buffer and rgba8_buffer.HasData():
                rgba8_data = rgba8_buffer.GetRGBA8Data()
                if rgba8_data is not None:
                    print(f'Time: {ch_time:.2f}s - RGBA8 buffer: {rgba8_buffer.Width}x{rgba8_buffer.Height}, Frame: {manager.GetNumUpdates()}')
                    

            r8_buffer = cam.GetMostRecentR8Buffer()
            if r8_buffer and r8_buffer.HasData():
                r8_data = r8_buffer.GetR8Data()
                if r8_data is not None:
                    print(f'Time: {ch_time:.2f}s - R8 buffer (post-resize): {r8_buffer.Width}x{r8_buffer.Height}, Frame: {manager.GetNumUpdates()}')
                    
        frame_count +=1


    t2 = time.time()
    print("----------------------------------------------------------------------------")
    print(f"Simulation finished. Sim time: {ch_time:.2f}s, Wall time: {t2 - t1:.2f}s")
    if save:
        print(f"Images saved to '{os.path.abspath(out_dir)}rgb/' and '{os.path.abspath(out_dir)}gray/'")


if __name__ == "__main__":
    main()