import os
import math
import pychrono as chrono
import pychrono.sensor as sens


def main():
    
    
    
    
    chrono.SetDataPath(os.path.join(os.path.dirname(__file__), "data/"))

    
    
    
    system = chrono.ChSystemNSC()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    system.Set_G_acc(chrono.Vector(0, 0, -9.81))

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    ground.SetPos(chrono.Vector(0, 0, 0))
    ground.SetCollide(False)
    system.AddBody(ground)

    
    
    
    
    obj_file = os.path.join(os.path.dirname(__file__), "mesh.obj")
    if not os.path.isfile(obj_file):
        
        obj_file = chrono.GetChronoDataFile("sensor/tri.obj")
        if not os.path.isfile(obj_file):
            raise FileNotFoundError(
                "Could not locate an .obj mesh. "
                "Please provide a Wavefront file named 'mesh.obj' in the script directory."
            )

    mesh_body = chrono.ChBody()
    mesh_body.SetFixed(True)                
    mesh_body.SetPos(chrono.Vector(0, 0, 0))
    mesh_body.SetCollide(False)             

    
    tri_mesh = chrono.ChTriangleMeshConnected()
    tri_mesh.LoadWavefront(obj_file, normalized=True, load_normals=True)

    
    vis_shape = chrono.ChVisualShapeTriangleMesh()
    vis_shape.SetMesh(tri_mesh)
    vis_shape.SetColor(chrono.ChColor(0.8, 0.5, 0.2))   
    mesh_body.AddVisualShape(vis_shape)
    system.AddBody(mesh_body)

    
    
    
    sensor_parent = chrono.ChBody()
    sensor_parent.SetMass(1.0)
    sensor_parent.SetBodyFixed(True)   
    sensor_parent.SetCollide(False)
    sensor_parent.SetPos(chrono.Vector(5.0, 0.0, 1.5))   
    system.AddBody(sensor_parent)

    
    
    
    manager = sens.ChSensorManager(system)

    
    
    
    update_rate = 30.0                     
    max_range = 50.0                       
    horizontal_fov = 360.0                 
    vertical_fov = 60.0                    
    angular_res = 1.0                       

    lidar_params = sens.ChLidarSensorParameters()
    lidar_params.SetMaxRange(max_range)
    lidar_params.SetLidarReturnMode(sens.LidarReturnMode_STRONGEST_RETURN)
    lidar_params.SetScanAngle(horizontal_fov)
    lidar_params.SetVerticalScanAngle(vertical_fov)
    lidar_params.SetAngularResolution(angular_res)
    lidar_params.SetPayloadSampleRadius(0.05)   
    lidar_params.SetAttenuation(0.025)          

    
    noise = sens.ChNoiseGauss()
    noise.SetMean(0.0)
    noise.SetSTD(0.01)
    lidar_params.SetNoiseModel(noise)

    
    sensor_offset = chrono.ChFrameD(chrono.Vector(0, 0, 0), chrono.Q_from_AngZ(0))

    
    lidar = sens.ChLidarSensor(sensor_parent, update_rate, sensor_offset, lidar_params)

    
    lidar.SetRenderActive(True)
    lidar.SetDrawSensorMesh(True)

    
    
    
    
    manager.SetLidarRenderMode(sens.LidarRenderMode_PointCloud)

    
    lidar.SetDataSavingActive(True)
    lidar.SetDataSaveFolder("./lidar_output")
    lidar.SetDataSaveBaseFileName("lidar")
    lidar.SetDataSaveFormat(sens.DataFileFormat_CSV)

    
    manager.AddSensor(lidar)

    
    
    
    step_size = 1e-3          
    total_time = 5.0          
    orbit_radius = 5.0        
    orbit_height = 1.5        
    orbit_speed = 1.0         

    t = 0.0
    while t < total_time:
        
        manager.Update()

        
        angle = orbit_speed * t
        x = orbit_radius * math.cos(angle)
        y = orbit_radius * math.sin(angle)
        sensor_parent.SetPos(chrono.Vector(x, y, orbit_height))

        
        
        
        
        

        
        system.DoStepDynamics(step_size)

        
        data = lidar.GetMostRecentDataBuffer()
        if data:
            points = data.Points          
            print(f"Time {t:.3f} | Points: {len(points)}")
            
            for i, pt in enumerate(points[:5]):
                print(f"  [{i}] ({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f})")
        else:
            print(f"Time {t:.3f} | No data (sensor not updated yet)")

        t += step_size

    print("\nSimulation finished – lidar data saved in ./lidar_output/")

if __name__ == "__main__":
    main()