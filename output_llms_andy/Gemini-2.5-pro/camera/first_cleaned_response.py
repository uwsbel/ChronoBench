import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import os
import math
import numpy as np




try:
    chrono.SetChronoDataPath(os.environ['CHRONO_DATA_DIR'])
except KeyError:
    print("Warning: CHRONO_DATA_DIR environment variable not set.")
    
    
    
    
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir_guess = os.path.join(script_dir, '..', '..', 'data') 
    if os.path.exists(os.path.join(data_dir_guess, 'models', 'utah_teapot.obj')):
         chrono.SetChronoDataPath(data_dir_guess)
    else:
        print("Fallback data path not found. Please set CHRONO_DATA_DIR or modify script.")
        exit(1)




print("Initializing Chrono system...")

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, -9.81, 0)) 






print("Creating fixed mesh body...")

mesh_body = chrono.ChBody()
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0)) 
mesh_body.SetBodyFixed(True) 
system.Add(mesh_body)


obj_filename = chrono.GetChronoDataFile("models/utah_teapot.obj") 
if not os.path.exists(obj_filename):
    print(f"Error: OBJ file not found at {obj_filename}")
    print("Please ensure 'utah_teapot.obj' (or your chosen OBJ) is in the Chrono data/models directory,")
    print("or provide the correct path.")
    exit(1)

trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(obj_filename, False, True) 
trimesh.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(0.02)) 


mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(trimesh)
mesh_shape.SetName("Teapot Mesh")
mesh_shape.SetColor(chrono.ChColor(0.3, 0.5, 0.8))
mesh_body.AddVisualShape(mesh_shape)




material_mesh = chrono.ChContactMaterialNSC() 
mesh_body.AddCollisionShape(chrono.ChCollisionShapeTriangleMesh(material_mesh, trimesh, False, False, 0.0))
mesh_body.EnableCollision(True)



print("Setting up sensor manager...")
manager = sens.ChSensorManager(system)
manager.SetVerbose(False) 



camera_host_body = chrono.ChBody()
camera_host_body.SetPos(chrono.ChVectorD(2, 1, 0)) 
camera_host_body.SetBodyFixed(False) 
system.Add(camera_host_body)



print("Adding camera sensor...")
camera_update_rate = 30.0  
image_width = 1280
image_height = 720
hfov = 1.4  




camera_offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)

camera = sens.ChCameraSensor(
    camera_host_body,       
    camera_update_rate,     
    camera_offset_pose,     
    image_width,            
    image_height,           
    hfov                    
)
camera.SetName("OrbitingCamera")
camera.SetLag(0) 
camera.SetCollectionWindow(0) 



noise_model = sens.ChNoiseNormal(image_width, image_height, 0.0, 0.02, 0.02, 0.02) 
camera.PushFilter(sens.ChFilter สิงห์Noise(noise_model, "GaussianNoise"))



camera.PushFilter(sens.ChFilterVisualize(image_width, image_height, "Camera Output", False))


manager.AddSensor(camera)








print("Setting up Irrlicht visualizer...")
vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Simulation: Orbiting Camera on Fixed Mesh")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 2, 4), chrono.ChVectorD(0, 0, 0)) 
vis.AddTypicalLights()


grid = chrono.ChVisualShapeBox(20, 0.1, 20)
grid.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"), 20, 20)
grid_body = chrono.ChBody()
grid_body.SetPos(chrono.ChVectorD(0, -0.55, 0))
grid_body.SetBodyFixed(True)
grid_body.AddVisualShape(grid)
system.Add(grid_body)





print("Starting simulation loop...")
time_step = 0.01
simulation_time = 10.0  


orbit_radius = 2.5
orbit_speed = 0.5  
orbit_height = 0.8
orbit_center = mesh_body.GetPos() 


print_interval = int(1.0 / (camera_update_rate * time_step)) 
frame_count = 0

while vis.Run():
    current_time = system.GetChTime()
    if current_time > simulation_time:
        break

    vis.BeginScene(True, True, chrono.ChColor(0.1, 0.1, 0.1))
    vis.Render()
    

    
    angle = orbit_speed * current_time

    
    cam_x = orbit_center.x + orbit_radius * math.cos(angle)
    cam_y = orbit_center.y + orbit_height
    cam_z = orbit_center.z + orbit_radius * math.sin(angle)
    camera_pos = chrono.ChVectorD(cam_x, cam_y, cam_z)
    camera_host_body.SetPos(camera_pos)

    
    
    
    
    
    
    
    
    
    rot_matrix = chrono.ChMatrix33D()
    world_up_vector = chrono.ChVectorD(0, 1, 0) 
    
    
    
    rot_matrix.Set_A_look_at_point(camera_pos, orbit_center, world_up_vector)
    camera_host_body.SetRot(rot_matrix)


    
    manager.Update()
    system.DoStepDynamics(time_step)

    
    
    rgba_buffer = camera.GetMostRecentBufferRGBA8()
    if rgba_buffer and rgba_buffer.HasData():
        
        if frame_count % (int(camera_update_rate / 2)) == 0 : 
            data = rgba_buffer.GetRGBA8Data() 
            
            
            
            
            print(f"\n--- Time: {current_time:.2f}s ---")
            print(f"Camera: {camera.GetName()}")
            print(f"Buffer Dimensions: {rgba_buffer.Width}x{rgba_buffer.Height}")
            print(f"First 12 pixel components (RGBA RGBA RGBA...):")
            print(list(data[:12])) 
            
            
            
            
            

    vis.EndScene()
    frame_count += 1




print("Simulation finished.")
if vis.GetDevice():
    vis.GetDevice().closeDevice()