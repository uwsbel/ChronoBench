import math
import time

import pychrono.core as chrono
import pychrono.sensor as sens







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
    mmesh.LoadWavefrontMesh(
        chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj"),
        False, True
    )
    mmesh.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33d(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    intensity = 1.0
    
    manager.scene.AddPointLight(
        chrono.ChVectorF(2, 2.5, 100),
        chrono.ChColor(intensity, intensity, intensity),
        500.0
    )
    
    manager.scene.AddAreaLight(
        chrono.ChVectorF(0, 0, 4),
        chrono.ChColor(intensity, intensity, intensity),
        500.0,
        chrono.ChVectorF(1, 0, 0),
        chrono.ChVectorF(0, -1, 0)
    )

    
    
    
    
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(-7, 0, 2),
        chrono.Q_from_AngAxis(2.0, chrono.ChVectorD(0, 1, 0))
    )

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

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width, image_height,
                                              "Before Grayscale Filter"))

    cam.PushFilter(sens.ChFilterRGBA8Access())

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "rgb/"))

    cam.PushFilter(sens.ChFilterGrayscale())

    if vis:
        cam.PushFilter(sens.ChFilterVisualize(image_width // 2,
                                              image_height // 2,
                                              "Grayscale Image"))

    if save:
        cam.PushFilter(sens.ChFilterSave(out_dir + "gray/"))

    cam.PushFilter(sens.ChFilterImageResize(image_width // 2,
                                            image_height // 2))
    cam.PushFilter(sens.ChFilterR8Access())

    manager.AddSensor(cam)

    
    
    
    orbit_radius = 10.0          
    orbit_rate = 0.1             
    ch_time = 0.0

    t_wall_start = time.time()

    while ch_time < end_time:
        
        cam.SetOffsetPose(
            chrono.ChFrameD(
                chrono.ChVectorD(-orbit_radius * math.cos(ch_time * orbit_rate),
                                 -orbit_radius * math.sin(ch_time * orbit_rate),
                                 1),
                chrono.Q_from_AngAxis(ch_time * orbit_rate,
                                      chrono.ChVectorD(0, 0, 1))
            )
        )

        rgba8_buffer = cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer and rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(f'RGBA8 buffer received. Resolution: '
                  f'{rgba8_buffer.Width} x {rgba8_buffer.Height}')
            print(f'First pixel: {rgba8_data[0, 0, :]}')

        manager.Update()                    
        mphysicalSystem.DoStepDynamics(step_size)

        ch_time = mphysicalSystem.GetChTime()

    print(f"Simulated time = {end_time} s | "
          f"wall-clock time = {time.time() - t_wall_start:.2f} s")





if __name__ == "__main__":
    main()