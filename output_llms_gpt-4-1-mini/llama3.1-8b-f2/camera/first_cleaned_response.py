import pychrono.core as chrono
import pychrono.sensor as sens

import math
import time


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    
    
    
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "models/lowpoly/car/car.obj"), False, True)
    
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Car Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)

    
    
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-5, 0, 2), chrono.QuatFromAngleAxis(2, chrono.ChVector3d(0, 1, 0)))
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
        cam.PushFilter(sens.ChFilterVisualize(
            image_width, image_height, "Before Grayscale Filter"))

    
    
    cam.PushFilter(sens.ChFilterGrayscale())

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            int(image_width / 2), int(image_height / 2), "Grayscale Image"))

    
    cam.PushFilter(sens.ChFilterThreshold(0.2, 1.0))

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            int(image_width / 2), int(image_height / 2), "Thresholded Image"))

    
    
    cam.PushFilter(sens.ChFilterHistogram(10, 0, 1))

    
    if vis:
        cam.PushFilter(sens.ChFilterVisualize(
            100, 100, "Histogram"))

    
    manager.AddSensor(cam)

    
    
    
    orbit_radius = 5
    orbit_rate = 0.2
    ch_time = 0.0

    t1 = time.time()

    while (ch_time < end_time):
        cam.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(ch_time * orbit_rate), -
                             orbit_radius * math.sin(ch_time * orbit_rate), 1),
            chrono.QuatFromAngleAxis(ch_time * orbit_rate, chrono.ChVector3d(0, 0, 1))))

        
        cam_buffer = cam.GetMostRecentCameraBuffer()
        if cam_buffer.HasData():
            buffer_data = cam_buffer.GetBuffer()
            print('Camera buffer data: ', buffer_data.shape, buffer_data.mean(),
                  buffer_data.std(), 'FPS: ', update_rate / (time.time() - t1))
            t1 = time.time()

        
        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(step_size)

        
        ch_time = mphysicalSystem.GetChTime()

    return 0












noise_model = "NONE"              



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

main()