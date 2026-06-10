import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math
import os




SIMULATION_STEP_SIZE = 1e-3        
SIMULATION_END_TIME  = 20.0        


CAMERA_WIDTH        = 1280
CAMERA_HEIGHT       = 720
CAMERA_FOV          = math.pi / 3  
CAMERA_UPDATE_RATE  = 30           
CAMERA_ORBIT_RADIUS = 5.0          
CAMERA_ORBIT_SPEED  = 0.5          


MESH_OBJ_FILE = "shape.obj"        




def ensure_mesh_file(path: str) -> None:
    
    if os.path.isfile(path):
        return
    print(f"[INFO] '{path}' not found – generating a placeholder tetrahedron.")
    content = 
    with open(path, "w") as f:
        f.write(content)




system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))

print("[INFO] Chrono system initialised.")




ensure_mesh_file(MESH_OBJ_FILE)


trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(MESH_OBJ_FILE, True, True)
trimesh.Transform(
    chrono.ChVectorD(0, 0, 0),
    chrono.ChMatrix33D(chrono.Q_from_AngAxis(0, chrono.ChVectorD(0, 1, 0)))
)


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
mesh_body.SetName("MeshBody")


vis_mesh = chrono.ChTriangleMeshShape()
vis_mesh.SetMesh(trimesh)
vis_mesh.SetName("MeshVisual")
vis_mesh.SetMutable(False)
mesh_body.AddVisualShape(vis_mesh)


mesh_material = chrono.ChMaterialSurfaceNSC()
collision_mesh = chrono.ChTriangleMeshShape()
collision_mesh.SetMesh(trimesh)

system.Add(mesh_body)
print(f"[INFO] Mesh body '{mesh_body.GetName()}' added to system (fixed={mesh_body.GetBodyFixed()}).")




manager = sens.ChSensorManager(system)


manager.scene.AddPointLight(
    chrono.ChVectorF(3, 3, 3),   
    chrono.ChColor(1, 1, 1),     
    50.0                          
)
manager.scene.AddPointLight(
    chrono.ChVectorF(-3, 3, -3),
    chrono.ChColor(0.8, 0.8, 1.0),
    30.0
)

print("[INFO] Sensor manager created.")





initial_cam_pos = chrono.ChVectorD(CAMERA_ORBIT_RADIUS, 1.0, 0.0)
look_at         = chrono.ChVectorD(0, 0, 0)
up              = chrono.ChVectorD(0, 1, 0)


cam_z = (look_at - initial_cam_pos).GetNormalized()       
cam_x = up.Cross(cam_z).GetNormalized()                   
cam_y = cam_z.Cross(cam_x).GetNormalized()                

rot_matrix = chrono.ChMatrix33D()
rot_matrix.Set_A_axis(cam_x, cam_y, cam_z)
cam_rotation = chrono.ChQuaternionD()
cam_rotation.Set_A_Rmatrix(rot_matrix)

camera_offset_pose = chrono.ChFrameD(initial_cam_pos, cam_rotation)




camera = sens.ChCameraSensor(
    mesh_body,            
    CAMERA_UPDATE_RATE,   
    camera_offset_pose,   
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV
)
camera.SetName("OrbitCamera")
camera.SetLag(0.0)           
camera.SetCollectionWindow(1.0 / CAMERA_UPDATE_RATE)




noise_model = sens.ChNoiseNormal(
    chrono.ChVectorF(0, 0, 0),          
    chrono.ChVectorF(0.01, 0.01, 0.01)  
)
camera.PushFilter(sens.ChFilterCameraNoiseConstNormal(0.0, 0.02))


camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Raw Camera Feed"))


camera.PushFilter(sens.ChFilterRGBA8Access())


camera.PushFilter(sens.ChFilterGrayscale())


camera.PushFilter(sens.ChFilterVisualize(CAMERA_WIDTH, CAMERA_HEIGHT, "Greyscale Feed"))


camera.PushFilter(sens.ChFilterR8Access())


manager.AddSensor(camera)

print("[INFO] Camera sensor added to manager with noise + visualisation filters.")




sim_time   = 0.0
step_count = 0
orbit_angle = 0.0

print("[INFO] Starting simulation loop …")
print(f"       step size = {SIMULATION_STEP_SIZE} s  |  end time = {SIMULATION_END_TIME} s")
print("-" * 60)

while sim_time < SIMULATION_END_TIME:

    
    
    
    orbit_angle = CAMERA_ORBIT_SPEED * sim_time   

    cam_x_pos = CAMERA_ORBIT_RADIUS * math.cos(orbit_angle)
    cam_z_pos = CAMERA_ORBIT_RADIUS * math.sin(orbit_angle)
    cam_y_pos = 1.5 * math.sin(orbit_angle * 0.3)  

    new_cam_pos = chrono.ChVectorD(cam_x_pos, cam_y_pos, cam_z_pos)
    look_at     = chrono.ChVectorD(0, 0, 0)

    
    forward = (look_at - new_cam_pos).GetNormalized()
    world_up = chrono.ChVectorD(0, 1, 0)

    right   = world_up.Cross(forward)
    if right.Length() < 1e-6:          
        world_up = chrono.ChVectorD(0, 0, 1)
        right    = world_up.Cross(forward)
    right = right.GetNormalized()
    cam_up  = forward.Cross(right).GetNormalized()

    new_rot_mat = chrono.ChMatrix33D()
    new_rot_mat.Set_A_axis(right, cam_up, forward)
    new_quat = chrono.ChQuaternionD()
    new_quat.Set_A_Rmatrix(new_rot_mat)

    new_offset = chrono.ChFrameD(new_cam_pos, new_quat)
    camera.SetOffsetPose(new_offset)

    
    
    
    manager.Update()

    
    
    
    system.DoStepDynamics(SIMULATION_STEP_SIZE)

    
    
    
    
    if step_count % int(1.0 / SIMULATION_STEP_SIZE) == 0:

        rgba_buffer = camera.GetMostRecentRGBA8Buffer()
        grey_buffer = camera.GetMostRecentR8Buffer()

        rgba_info = "unavailable"
        grey_info = "unavailable"

        if rgba_buffer and rgba_buffer.HasData():
            rgba_data  = rgba_buffer.GetRGBA8Data()          
            rgba_info  = (f"shape={rgba_data.shape}  "
                          f"dtype={rgba_data.dtype}  "
                          f"mean(R,G,B,A)=("
                          f"{rgba_data[:,:,0].mean():.3f}, "
                          f"{rgba_data[:,:,1].mean():.3f}, "
                          f"{rgba_data[:,:,2].mean():.3f}, "
                          f"{rgba_data[:,:,3].mean():.3f})")

        if grey_buffer and grey_buffer.HasData():
            grey_data = grey_buffer.GetChar8Data()            
            grey_info = (f"shape={grey_data.shape}  "
                         f"dtype={grey_data.dtype}  "
                         f"mean={grey_data.mean():.3f}  "
                         f"max={grey_data.max()}  "
                         f"min={grey_data.min()}")

        print(f"[t={sim_time:6.3f}s | step={step_count:6d}]")
        print(f"  Camera pos : ({cam_x_pos:.3f}, {cam_y_pos:.3f}, {cam_z_pos:.3f})  "
              f"orbit angle = {math.degrees(orbit_angle):.1f}°")
        print(f"  RGBA buffer: {rgba_info}")
        print(f"  Grey buffer: {grey_info}")
        print()

    sim_time   += SIMULATION_STEP_SIZE
    step_count += 1




print("-" * 60)
print(f"[INFO] Simulation finished.  Total steps: {step_count}")
print(f"[INFO] Simulated time: {sim_time:.4f} s")