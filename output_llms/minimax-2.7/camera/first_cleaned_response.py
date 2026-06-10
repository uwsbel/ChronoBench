import math
import numpy as np               
import pychrono as chrono
import pychrono.sensor as sens

def main():
    
    
    
    system = chrono.ChSystemNSC()
    system.SetTimestepperType(chrono.ChTimestepper.Type_Euler)
    system.SetStepSize(1.0e-3)            
    system.Set_G_acc(chrono.ChVectorD(0, 0, 0))   

    
    
    
    mesh_file = "mesh.obj"   
    mesh = chrono.ChTriangleMesh()
    try:
        mesh.LoadWavefrontMesh(mesh_file, True, True)
        print(f"[INFO] Loaded mesh from '{mesh_file}'")
    except Exception as e:
        print(f"[ERROR] Could not load mesh: {e}")
        return

    
    mesh_body = chrono.ChBody()
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.SetBodyFixed(True)                     
    vis_shape = chrono.ChTriangleMeshShape()
    vis_shape.SetMesh(mesh)
    mesh_body.AddVisualShape(vis_shape)
    system.AddBody(mesh_body)

    
    
    
    cam_body = chrono.ChBody()
    cam_body.SetPos(chrono.ChVectorD(5, 0, 0))        
    cam_body.SetBodyFixed(False)                      
    system.AddBody(cam_body)

    
    
    
    update_rate = 30               
    camera_offset = chrono.ChFrame(chrono.ChVectorD(0, 0, 0))   
    width, height = 640, 480

    camera = sens.ChCameraSensor(
        cam_body,               
        update_rate,            
        camera_offset,          
        width, height,          
        "Camera"                
    )

    
    noise = sens.ChNoiseGaussian(0.0, 0.02)
    camera.PushFilter(noise)

    
    camera.PushFilter(sens.ChFilterRGBA8Access())

    
    camera.PushFilter(sens.ChFilterVisualize())

    
    
    
    manager = sens.ChSensorManager(system)
    manager.AddSensor(camera)

    
    
    
    orbit_radius = 5.0            
    orbit_speed = 0.5             
    sim_time = 0.0
    end_time = 5.0                
    time_step = system.GetStepSize()

    
    forward_local = chrono.ChVectorD(0, 0, -1)

    
    
    
    step = 0
    while sim_time < end_time:
        
        system.DoStepDynamics(time_step)

        
        manager.Update()

        
        angle = sim_time * orbit_speed
        x = orbit_radius * math.cos(angle)
        z = orbit_radius * math.sin(angle)
        cam_body.SetPos(chrono.ChVectorD(x, 0, z))

        
        dir_to_origin = -cam_body.GetPos()          
        dir_to_origin.Normalize()
        q = chrono.Q_from_Vect_To_Vect(forward_local, dir_to_origin)
        cam_body.SetRot(q)

        
        if camera.IsNewDataAvailable():
            
            frame = camera.GetMostRecentData()
            
            print(f"Step {step:06d} | SimTime {sim_time:.3f} s | "
                  f"Buffer shape: {frame.shape}, dtype: {frame.dtype}")
            print(f"  First pixel (RGBA): {frame[0, 0, :]}")
            print(f"  Sample first 10 values: {frame.ravel()[:10]}")

        sim_time += time_step
        step += 1

    print("\n[INFO] Simulation finished.")

if __name__ == "__main__":
    main()