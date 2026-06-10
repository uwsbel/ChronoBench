import os
import math
import numpy as np

import pychrono as chrono
import pychrono.sensor as sens









OBJ_FILE = chrono.GetChronoDataFile("sensor/box.obj")

OUTPUT_DIR = "SENSOR_OUTPUT/lidar_orbit"
os.makedirs(OUTPUT_DIR, exist_ok=True)

simulation_time = 10.0
step_size = 1.0e-3


lidar_update_rate = 10.0          
horizontal_samples = 1024
vertical_samples = 64
horizontal_fov = 2.0 * math.pi    
vertical_fov_upper = math.radians(10.0)
vertical_fov_lower = math.radians(-30.0)
max_lidar_range = 100.0


orbit_radius = 6.0
orbit_height = 2.0
orbit_angular_speed = 0.5         
mesh_center = chrono.ChVector3d(0.0, 0.0, 0.0)







def get_orbiting_lidar_pose(t):
    angle = orbit_angular_speed * t

    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height

    pos = chrono.ChVector3d(x, y, z)

    
    
    yaw = angle + math.pi

    
    dist_horizontal = orbit_radius
    pitch = math.atan2(orbit_height, dist_horizontal)

    q_yaw = chrono.QuatFromAngleAxis(yaw, chrono.ChVector3d(0, 0, 1))
    q_pitch = chrono.QuatFromAngleAxis(pitch, chrono.ChVector3d(0, 1, 0))

    rot = q_yaw * q_pitch

    return chrono.ChFramed(pos, rot)






system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))





if not os.path.isfile(OBJ_FILE):
    raise FileNotFoundError(
        f"OBJ file not found: {OBJ_FILE}\n"
        "Set OBJ_FILE to a valid Wavefront .obj mesh."
    )

mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(OBJ_FILE, False, True)

mesh_shape = chrono.ChVisualShapeTriangleMesh()
mesh_shape.SetMesh(mesh)
mesh_shape.SetName("fixed_obj_mesh")
mesh_shape.SetMutable(False)
mesh_shape.SetColor(chrono.ChColor(0.65, 0.65, 0.65))

mesh_body = chrono.ChBody()
mesh_body.SetName("Fixed OBJ Mesh Body")
mesh_body.SetFixed(True)
mesh_body.SetPos(mesh_center)
mesh_body.AddVisualShape(mesh_shape)

system.Add(mesh_body)





use_irrlicht = True

if use_irrlicht:
    vis = chrono.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("PyChrono Lidar Orbit Around OBJ Mesh")
    vis.Initialize()
    vis.AddSkyBox()
    vis.AddCamera(
        chrono.ChVector3d(8, -8, 5),
        chrono.ChVector3d(0, 0, 0)
    )
    vis.AddTypicalLights()
else:
    vis = None






manager = sens.ChSensorManager(system)


manager.scene.AddPointLight(
    chrono.ChVector3f(5, -5, 5),
    chrono.ChColor(1.0, 1.0, 1.0),
    500.0
)

manager.scene.AddPointLight(
    chrono.ChVector3f(-5, 5, 5),
    chrono.ChColor(1.0, 1.0, 1.0),
    300.0
)





initial_lidar_pose = get_orbiting_lidar_pose(0.0)

lidar = sens.ChLidarSensor(
    mesh_body,                 
    lidar_update_rate,          
    initial_lidar_pose,         
    horizontal_samples,         
    vertical_samples,           
    horizontal_fov,             
    vertical_fov_upper,         
    vertical_fov_lower,         
    max_lidar_range             
)

lidar.SetName("Orbiting Lidar")
lidar.SetLag(0.0)
lidar.SetCollectionWindow(1.0 / lidar_update_rate)






lidar.PushFilter(sens.ChFilterPCfromDepth())



lidar.PushFilter(
    sens.ChFilterLidarNoiseXYZI(
        0.01,   
        0.001,  
        0.001,  
        0.01    
    )
)


lidar.PushFilter(
    sens.ChFilterVisualizePointCloud(
        640,
        480,
        1.0,
        "Lidar Point Cloud"
    )
)


lidar.PushFilter(
    sens.ChFilterSavePtCloud(OUTPUT_DIR)
)


lidar.PushFilter(
    sens.ChFilterXYZIAccess()
)

manager.AddSensor(lidar)






print("Starting simulation...")
print(f"OBJ mesh: {OBJ_FILE}")
print(f"Lidar output directory: {OUTPUT_DIR}")

while system.GetChTime() < simulation_time:
    time = system.GetChTime()

    
    lidar_pose = get_orbiting_lidar_pose(time)
    lidar.SetOffsetPose(lidar_pose)

    
    manager.Update()

    
    xyzi_buffer = lidar.GetMostRecentXYZIBuffer()

    if xyzi_buffer.HasData():
        xyzi = xyzi_buffer.GetXYZI()

        
        
        num_points = xyzi.shape[0]

        print(f"\nTime: {time:.3f} s")
        print(f"Number of lidar points: {num_points}")

        if num_points > 0:
            print("First five XYZI points:")
            print(xyzi[:5])
    else:
        print(f"Time: {time:.3f} s | No new lidar data")

    
    if vis is not None:
        if not vis.Run():
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

    
    system.DoStepDynamics(step_size)

print("Simulation complete.")