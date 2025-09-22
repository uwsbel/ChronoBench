import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import os
import math

def main():
    
    chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, 0))

    
    ground = chrono.ChBody()
    ground.SetFixed(True)
    system.Add(ground)

    
    mesh_path = chrono.GetChronoDataFile("models/cube.obj")  
    mesh = chrono.ChVisualShapeModelFile(mesh_path)
    ground.AddVisualShape(mesh)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3f(2, 2, 2), chrono.ChColor(1, 1, 1), 500)

    
    orbit_radius = 3.0
    orbit_height = 1.0
    angular_speed = 0.1  

    
    camera_body = chrono.ChBody()
    camera_body.SetPos(chrono.ChVector3d(orbit_radius, 0, orbit_height))
    system.Add(camera_body)

    
    offset_pose = chrono.ChFramed()
    camera = sens.ChCameraSensor(
        camera_body,          
        0.1,                  
        offset_pose,          
        1280,                 
        720,                  
        chrono.CH_PI / 3      
    )
    camera.SetName("Orbiting Camera")
    camera.PushFilter(sens.ChFilterRGBA8Access())
    camera.PushFilter(sens.ChFilterVisualize(1280, 720, "Camera Output"))
    camera.PushFilter(sens.ChFilterNoiseNormal(0.0, 0.02))  

    manager.AddSensor(camera)

    
    step_size = 0.01
    time = 0
    while time < 10:
        
        angle = angular_speed * time
        cam_x = orbit_radius * math.cos(angle)
        cam_y = orbit_radius * math.sin(angle)
        camera_body.SetPos(chrono.ChVector3d(cam_x, cam_y, orbit_height))
        camera_body.SetRot(chrono.QuatFromAngleAxis(-angle, chrono.ChVector3d(0, 0, 1)))

        
        manager.Update()

        
        rgba8_data = camera.GetMostRecentRGBA8Buffer()
        if rgba8_data.HasData():
            buf = rgba8_data.GetRGBA8Data()
            print(f"Time: {time:.2f}s | Camera buffer: "
                  f"Avg R: {np.mean(buf[:,:,0]):.1f}, "
                  f"G: {np.mean(buf[:,:,1]):.1f}, "
                  f"B: {np.mean(buf[:,:,2]):.1f}, "
                  f"Size: {buf.shape}")

        
        system.DoStepDynamics(step_size)
        time += step_size

if __name__ == "__main__":
    main()