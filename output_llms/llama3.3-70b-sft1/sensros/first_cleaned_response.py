import pychrono as ch
import pychrono.sensor as sens

import pychrono.ros as chros

import time
import math


def main():
    
    
    
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0,0,-9.81))

    
    
    
    mmesh = ch.ChTriangleMeshConnected()
    mmesh.LoadWavefrontMesh(ch.GetChronoDataFile(
        "vehicle/hmmwv/hmmwv_chassis.obj"), False, True)
    mmesh.Transform(ch.ChVector3d(0, 0, 0), ch.ChMatrix33d(2))

    trimesh_shape = ch.ChVisualShapeTriangleMesh()
    trimesh_shape.SetMesh(mmesh)
    trimesh_shape.SetName("HMMWV Chassis Mesh")
    trimesh_shape.SetMutable(False)

    mesh_body = ch.ChBody()
    mesh_body.SetPos(ch.ChVector3d(0, 0, 0))
    mesh_body.AddVisualShape(trimesh_shape)
    mesh_body.SetFixed(False)
    mesh_body.SetMass(0)
    sys.Add(mesh_body)

    
    
    
    manager = sens.ChSensorManager(sys)
    intensity = 1.0
    manager.scene.AddPointLight(ch.ChVector3f(2, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(9, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(16, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)
    manager.scene.AddPointLight(ch.ChVector3f(23, 2.5, 100), ch.ChColor(intensity, intensity, intensity), 500.0)

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
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

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(
        mesh_body,              
        update_rate,            
        offset_pose,            
        horizontal_samples,     
        vertical_samples,       
        horizontal_fov,         
        max_vert_angle,
        min_vert_angle,         
        100  
    )
    lidar.SetName("Lidar Sensor")
    lidar.SetLag(lag)
    lidar.SetReturnMode(sens.LidarReturnMode_STRONGEST_RETURN)

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    gps = sens.ChGPSSensor(mesh_body,              
                           update_rate,            
                           offset_pose,            
                           gps_reference,
                           nois_deviation)
    gps.SetName("GPS Sensor")
    gps.SetLag(lag)

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    acc = sens.ChAccelerometerSensor(mesh_body,              
                                     update_rate,            
                                     offset_pose,            
                                     noise_none)
    acc.SetName("Accelerometer Sensor")
    acc.SetLag(lag)

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    gyro = sens.ChGyroscopeSensor(mesh_body,              
                                  update_rate,            
                                  offset_pose,            
                                  noise_none)
    gyro.SetName("Gyroscope Sensor")
    gyro.SetLag(lag)

    
    
    
    offset_pose = ch.ChFramed(ch.ChVector3d(-8, 0, 2),
                              ch.QuatFromAngleAxis(.2, ch.ChVector3d(0, 1, 0)))
    mag = sens.ChMagnetometerSensor(mesh_body,              
                                    update_rate,            
                                    offset_pose,            
                                    gps_reference,
                                    noise_none)
    mag.SetName("Magnetometer Sensor")
    mag.SetLag(lag)

    
    
    
    manager.AddSensor(cam)
    manager.AddSensor(lidar)
    manager.AddSensor(gps)
    manager.AddSensor(acc)
    manager.AddSensor(gyro)
    manager.AddSensor(mag)

    
    
    
    
    filter_graph = manager.GetFilterGraph(cam.GetName())

    
    gray_filter = sens.ChFilterGrayscale()
    filter_graph.AddFilter(gray_filter)

    
    resize_filter = sens.ChFilterImageResize()
    resize_filter.SetNewWidth(256)
    resize_filter.SetNewHeight(256)
    filter_graph.AddFilter(resize_filter)

    
    
    
    ros_manager = chros.ChROSPythonManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())

    
    
    
    
    
    
    
    cam_handler = chros.ChROSCameraHandler(
        filter_graph.GetOutput(),  
        "~/output/camera/data/image"  
    )
    cam_handler.SetFps(5)
    cam_handler.SetImageEncoding("rgb8")
    ros_manager.RegisterHandler(cam_handler)

    
    
    
    lidar_handler = chros.ChROSLidarHandler(
        lidar,  
        "~/output/lidar/data/pointcloud"  
    )
    lidar_handler.SetFps(5)
    ros_manager.RegisterHandler(lidar_handler)

    
    
    
    gps_handler = chros.ChROSGPSHandler(
        gps,  
        "~/output/gps/data"  
    )
    gps_handler.SetFps(5)
    ros_manager.RegisterHandler(gps_handler)

    
    
    
    acc_handler = chros.ChROSAccelerometerHandler(
        acc,  
        "~/output/accelerometer/data"  
    )
    acc_handler.SetFps(5)
    ros_manager.RegisterHandler(acc_handler)

    
    
    
    gyro_handler = chros.ChROSGyroscopeHandler(
        gyro,  
        "~/output/gyroscope/data"  
    )
    gyro_handler.SetFps(5)
    ros_manager.RegisterHandler(gyro_handler)

    
    
    
    mag_handler = chros.ChROSMagnetometerHandler(
        mag,  
        "~/output/magnetometer/data"  
    )
    mag_handler.SetFps(5)
    ros_manager.RegisterHandler(mag_handler)

    
    
    
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    sys.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    t1 = time.time()

    orbit_radius = 20
    orbit_rate = 0.2
    ch.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    ch.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)
    sys.SetSolverType(ch.ChSolver.Type_BARZILAIBORWEIN)
    sys.GetSolver().AsIterative().SetMaxIterations(150)
    sys.SetMaxPenetrationRecoverySpeed(4.0)

    render_time = 0

    while sys.GetChTime() < end_time:
        
        t = sys.GetChTime()

        
        driver_inputs = ch.ChVector3d(
            orbit_radius * math.cos(t * orbit_rate),
            orbit_radius * math.sin(t * orbit_rate),
            0.0
        )
        mesh_body.SetPos(ch.ChVector3d(
            orbit_radius * math.cos(t * orbit_rate),
            orbit_radius * math.sin(t * orbit_rate),
            0.1
        ))
        mesh_body.SetRot(ch.QuatFromAngleAxis(
            t * orbit_rate, ch.ChVector3d(0, 0, 1)))

        
        
        manager.Update()

        
        manager.Advance(step_size)

        
        sys.DoStepDynamics(step_size)

        
        render_time = time.time()
        sys_time = sys.GetChTime()
        time_diff = (sys_time - render_time) * step_size
        if time_diff > 0:
            time.sleep(time_diff)

    print("Sim time:", end_time, "Wall time:", time.time()-t1)







update_rate = 5.0


image_width = 1280
image_height = 720


fov = 1.408


lag = 0


exposure_time = 0


horizontal_samples = 4500
vertical_samples = 32


horizontal_fov = 2 * ch.CH_PI  
max_vert_angle = ch.CH_PI / 12
min_vert_angle = -ch.CH_PI / 6


return_mode = sens.LidarReturnMode_STRONGEST_RETURN


gps_reference = ch.ChVector3d(0, 0, 0)



gps_noise_model = "NONE"


nois_deviation = 0



noise_none = "NONE"






step_size = 1e-3


end_time = 100.0


out_dir = "SENSOR_OUTPUT/"

main()