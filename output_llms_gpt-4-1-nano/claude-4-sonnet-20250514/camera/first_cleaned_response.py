import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np
import math
import os

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.4)
    contact_material.SetRestitution(0.2)
    
    
    
    mesh_file = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")
    
    
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    
    
    mesh_asset = chrono.ChTriangleMeshConnected()
    mesh_asset.LoadWavefrontMesh(mesh_file, False, True)
    mesh_asset.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    
    
    mesh_shape = chrono.ChTriangleMeshShape()
    mesh_shape.SetMesh(mesh_asset)
    mesh_shape.SetName("mesh_shape")
    mesh_shape.SetStatic(True)
    mesh_body.AddVisualShape(mesh_shape)
    
    
    system.Add(mesh_body)
    
    
    camera_body = chrono.ChBody()
    camera_body.SetBodyFixed(False)
    camera_body.SetMass(1.0)
    camera_body.SetPos(chrono.ChVectorD(5, 2, 5))
    system.Add(camera_body)
    
    
    sensor_manager = sens.ChSensorManager(system)
    
    
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(10, 10, 10), 
                                      chrono.ChVectorD(1, 1, 1), 200)
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(-10, 10, -10), 
                                      chrono.ChVectorD(1, 1, 1), 200)
    
    
    camera_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                                        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    
    camera = sens.ChCameraSensor(
        camera_body,                    
        30,                            
        camera_offset_pose,            
        1280,                          
        720,                           
        chrono.CH_C_PI / 3            
    )
    
    
    
    camera.PushFilter(sens.ChFilterImageResize(640, 360))
    camera.PushFilter(sens.ChFilterGrayscale())
    camera.PushFilter(sens.ChFilterNoisePixelGaussian(0.0, 0.02))
    
    
    camera.PushFilter(sens.ChFilterVisualize(640, 360, "Camera View"))
    
    
    camera.PushFilter(sens.ChFilterSave())
    
    
    camera.SetName("Camera_Sensor")
    
    
    sensor_manager.AddSensor(camera)
    
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono - Mesh with Orbiting Camera")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(10, 5, 10))
    vis.AddTypicalLights()
    
    
    time_step = 0.01
    time_end = 10.0
    orbit_radius = 8.0
    orbit_speed = 0.5
    orbit_height = 3.0
    
    
    time = 0
    step_number = 0
    
    while vis.Run() and time < time_end:
        
        angle = time * orbit_speed
        camera_x = orbit_radius * math.cos(angle)
        camera_z = orbit_radius * math.sin(angle)
        camera_y = orbit_height + 2 * math.sin(angle * 2)  
        
        
        camera_body.SetPos(chrono.ChVectorD(camera_x, camera_y, camera_z))
        
        
        look_at = chrono.ChVectorD(0, 0, 0) - chrono.ChVectorD(camera_x, camera_y, camera_z)
        look_at.Normalize()
        
        
        up = chrono.ChVectorD(0, 1, 0)
        right = look_at % up
        right.Normalize()
        up = right % look_at
        up.Normalize()
        
        
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.Set_A_axis(right, up, -look_at)
        camera_body.SetRot(chrono.ChQuaternionD(rot_matrix))
        
        
        sensor_manager.Update()
        
        
        if camera.GetMostRecentBuffer():
            buffer = camera.GetMostRecentBuffer()
            if hasattr(buffer, 'GetBuffer'):
                buffer_data = buffer.GetBuffer()
                print(f"Step {step_number}: Camera buffer shape: {buffer_data.shape if hasattr(buffer_data, 'shape') else 'N/A'}")
                print(f"  Camera position: ({camera_x:.2f}, {camera_y:.2f}, {camera_z:.2f})")
                print(f"  Buffer data type: {type(buffer_data)}")
                if hasattr(buffer_data, 'shape') and len(buffer_data.shape) > 0:
                    print(f"  Buffer size: {buffer_data.size} pixels")
                    if buffer_data.size > 0:
                        print(f"  Pixel value range: [{buffer_data.min():.3f}, {buffer_data.max():.3f}]")
        
        
        system.DoStepDynamics(time_step)
        time += time_step
        step_number += 1
        
        
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    print(f"\nSimulation completed after {step_number} steps")
    print(f"Total simulation time: {time:.2f} seconds")

if __name__ == "__main__":
    main()