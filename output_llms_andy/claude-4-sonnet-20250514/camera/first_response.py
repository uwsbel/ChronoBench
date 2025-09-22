import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import numpy as np
import math
import os

def main():
    # Initialize the PyChrono environment
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    
    # Create the physical system
    system = chrono.ChSystemNSC()
    system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
    
    # Create a contact material
    contact_material = chrono.ChMaterialSurfaceNSC()
    contact_material.SetFriction(0.4)
    contact_material.SetRestitution(0.2)
    
    # Load triangular mesh from Wavefront .obj file
    # Using a sample mesh from ChronoData - you can replace with your own .obj file
    mesh_file = chrono.GetChronoDataFile("vehicle/hmmwv/hmmwv_chassis.obj")
    
    # Create a fixed body for the mesh
    mesh_body = chrono.ChBody()
    mesh_body.SetBodyFixed(True)
    mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
    mesh_body.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))
    
    # Create mesh asset
    mesh_asset = chrono.ChTriangleMeshConnected()
    mesh_asset.LoadWavefrontMesh(mesh_file, False, True)
    mesh_asset.Transform(chrono.ChVectorD(0, 0, 0), chrono.ChMatrix33D(1))
    
    # Add visual shape
    mesh_shape = chrono.ChTriangleMeshShape()
    mesh_shape.SetMesh(mesh_asset)
    mesh_shape.SetName("mesh_shape")
    mesh_shape.SetStatic(True)
    mesh_body.AddVisualShape(mesh_shape)
    
    # Add the mesh body to the system
    system.Add(mesh_body)
    
    # Create a movable body for the camera
    camera_body = chrono.ChBody()
    camera_body.SetBodyFixed(False)
    camera_body.SetMass(1.0)
    camera_body.SetPos(chrono.ChVectorD(5, 2, 5))
    system.Add(camera_body)
    
    # Create sensor manager
    sensor_manager = sens.ChSensorManager(system)
    
    # Set sensor manager parameters
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(10, 10, 10), 
                                      chrono.ChVectorD(1, 1, 1), 200)
    sensor_manager.scene.AddPointLight(chrono.ChVectorD(-10, 10, -10), 
                                      chrono.ChVectorD(1, 1, 1), 200)
    
    # Create camera sensor
    camera_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), 
                                        chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
    
    camera = sens.ChCameraSensor(
        camera_body,                    # Body to attach to
        30,                            # Update rate in Hz
        camera_offset_pose,            # Offset pose
        1280,                          # Image width
        720,                           # Image height
        chrono.CH_C_PI / 3            # Horizontal field of view
    )
    
    # Add filters to the camera
    # Add noise filter
    camera.PushFilter(sens.ChFilterImageResize(640, 360))
    camera.PushFilter(sens.ChFilterGrayscale())
    camera.PushFilter(sens.ChFilterNoisePixelGaussian(0.0, 0.02))
    
    # Add visualization filter (saves images to buffer)
    camera.PushFilter(sens.ChFilterVisualize(640, 360, "Camera View"))
    
    # Add data access filter to get buffer data
    camera.PushFilter(sens.ChFilterSave())
    
    # Set camera name
    camera.SetName("Camera_Sensor")
    
    # Add camera to sensor manager
    sensor_manager.AddSensor(camera)
    
    # Create Irrlicht visualization
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle("PyChrono - Mesh with Orbiting Camera")
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVectorD(10, 5, 10))
    vis.AddTypicalLights()
    
    # Simulation parameters
    time_step = 0.01
    time_end = 10.0
    orbit_radius = 8.0
    orbit_speed = 0.5
    orbit_height = 3.0
    
    # Simulation loop
    time = 0
    step_number = 0
    
    while vis.Run() and time < time_end:
        # Update camera position in orbit around the mesh
        angle = time * orbit_speed
        camera_x = orbit_radius * math.cos(angle)
        camera_z = orbit_radius * math.sin(angle)
        camera_y = orbit_height + 2 * math.sin(angle * 2)  # Adding some vertical movement
        
        # Set camera body position
        camera_body.SetPos(chrono.ChVectorD(camera_x, camera_y, camera_z))
        
        # Orient camera to look at the mesh (at origin)
        look_at = chrono.ChVectorD(0, 0, 0) - chrono.ChVectorD(camera_x, camera_y, camera_z)
        look_at.Normalize()
        
        # Calculate rotation to point camera at mesh
        up = chrono.ChVectorD(0, 1, 0)
        right = look_at % up
        right.Normalize()
        up = right % look_at
        up.Normalize()
        
        # Create rotation matrix
        rot_matrix = chrono.ChMatrix33D()
        rot_matrix.Set_A_axis(right, up, -look_at)
        camera_body.SetRot(chrono.ChQuaternionD(rot_matrix))
        
        # Update sensor manager
        sensor_manager.Update()
        
        # Get camera buffer data and print information
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
        
        # Advance the simulation
        system.DoStepDynamics(time_step)
        time += time_step
        step_number += 1
        
        # Update visualization
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
    
    print(f"\nSimulation completed after {step_number} steps")
    print(f"Total simulation time: {time:.2f} seconds")

if __name__ == "__main__":
    main()