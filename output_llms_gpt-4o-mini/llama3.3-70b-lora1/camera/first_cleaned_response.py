import pychrono as chrono
import pychrono.sensor as sens

import time
import math


def main():
    
    
    
    mphysicalSystem = chrono.ChSystemNSC()

    
    
    
    mmesh = chrono.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(chrono.GetChronoDataFile(
        "vehicle/mmarm/mmesh.obj"), False, True)
    
    mmesh.Transform(chrono.ChVector3d(0, 0, 0), chrono.ChMatrix33d(2))

    trimesh_shape = chrono.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("Sensable object TriMesh shape")
    trimesh_shape.SetMutable(False)

    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(True)
    mphysicalSystem.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(mphysicalSystem)
    
    offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-8, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    intensity = 1.0

    
    
    
    
    
    
    image_width = 1280
    image_height = 720
    title = "Camera Sensor"

    
    
    
    lens_focal_length = 0
    noise_model = sens.NoiseModel_NONE

    
    
    
    exposure_time = 0  
    lag = 0            

    
    
    
    lens_distortion = sens.LensDistortionNONE

    
    
    
    window_width = 1280
    window_height = 720
    granularity = 2
    cam = sens.ChCameraSensor(
        mesh_body,              
        image_width,            
        image_height,           
        title,                  
        offset_pose,            
        exposure_time,          
        lag,                    
    )
    cam.SetName("Camera Sensor")

    if (lens_focal_length > 0):
        cam.SetLensFocalLength(lens_focal_length)
    if (noise_model == sens.NoiseModel_GAUSSIAN):
        cam.SetNoiseModel(sens.GaussianNoiseModel())
    elif (noise_model == sens.NoiseModel_UNIFORM):
        cam.SetNoiseModel(sens.UniformNoiseModel())

    if (lens_distortion == sens.LensDistortionRADTAN):
        cam.SetLensedRadialTangentDistortion(0.5, 0.5)
    elif (lens_distortion == sens.LensDistortionFOV):
        cam.SetLensedFovDistortion(0.5, 0.5)

    
    cam.PushFilter(sens.ChFilterVisualize(
        window_width, window_height, granularity))

    
    graph = cam.GetVacuumFilterGraph()
    
    graph.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    
    manager.AddSensor(cam)

    
    
    
    
    cam.PushFilter(sens.ChFilterVisualize(
        window_width, window_height, granularity))

    
    graph = cam.GetVacuumFilterGraph()
    
    graph.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    
    
    
    
    graph_depth = cam.GetDepthFilterGraph()
    graph_depth.PushFilter(sens.ChFilterVisualize(
        image_width, image_height, 1))

    
    
    
    
    graph = cam.GetVacuumFilterGraph()
    
    

    
    
    
    manager.Initialize()

    
    
    
    orbit_radius = 8
    orbit_rate = 0.5
    cam_offset_pose = chrono.ChFramed(
        chrono.ChVector3d(-orbit_radius, 0, 1), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    t = 0
    orbit_angle = 0
    render_step_size = 1.0 / 60  
    render_step_number = 0
    ch_time = 0

    while (ch_time < 10):

        small_step = orbit_rate * render_step_size
        orbit_angle += small_step
        cam_offset_pose = chrono.ChFramed(
            chrono.ChVector3d(-orbit_radius * math.cos(orbit_angle), -orbit_radius * math.sin(orbit_angle), 1), chrono.QuatFromAngleAxis(orbit_angle, chrono.ChVector3d(0, 0, 1)))
        cam.SetOffsetPose(cam_offset_pose)

        
        rgba8_buffer = graph.GetMostRecentBuffer().GetRGBA8Buffer()
        depth16_buffer = graph_depth.GetMostRecentBuffer().GetDepth16Buffer()

        
        
        manager.Update()

        
        mphysicalSystem.DoStepDynamics(render_step_size)

        
        ch_time = mphysicalSystem.GetChTime()

        print("buffer width: ", rgba8_buffer.Width)
        print("buffer height: ", rgba8_buffer.Height)
        print("buffer data: ", rgba8_buffer.Data)
        print("\n")

    return 0


if __name__ == "__main__":
    main()