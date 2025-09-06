import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


chrono.SetChronoDataPath(os.path.join(os.environ.get('CHRONO_DATA_DIR', ''), 'data/'))


sim_time = 0
time_step = 0.01
orbit_radius = 3.0
orbit_height = 1.0
orbit_speed = 0.5  


sys = chrono.ChSystemNSC()
sys.SetGravity(chrono.ChVector3d(0, -9.81, 0))


ground_mat = chrono.ChContactMaterialNSC()
ground = chrono.ChBodyEasyBox(10, 0.1, 10, 1000, True, True, ground_mat)
ground.SetFixed(True)
ground.SetPos(chrono.ChVector3d(0, -1, 0))
ground.SetName("Ground")
sys.Add(ground)


mesh_body = chrono.ChBody()
mesh_body.SetFixed(True)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetName("MeshBody")


mesh_asset = chrono.ChVisualShapeModelFile()
mesh_asset.SetFilename(chrono.GetChronoDataFile("models/cube.obj"))  
mesh_asset.SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
mesh_body.AddVisualShape(mesh_asset)
sys.Add(mesh_body)


manager = sens.ChSensorManager(sys)
manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 0), chrono.ChColor(1, 1, 1), 1000)


camera_res = (1280, 720)
camera_fov = 1.0  
camera_update_rate = 30  
camera_offset = chrono.ChVector3d(0, 0, 0)


camera = sens.ChCameraSensor(
    mesh_body,              
    camera_update_rate,     
    chrono.ChFramed(        
        camera_offset,
        chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0))
    ),
    camera_res[0],          
    camera_res[1],          
    camera_fov              
)
camera.SetName("Orbit Camera")
camera.SetLag(0.1)  


noise_model = sens.ChNoiseNormal(0.0, 0.1)  
camera.PushFilter(sens.ChFilterCameraNoise(noise_model))


camera.PushFilter(sens.ChFilterVisualize(camera_res[0], camera_res[1], "Camera View"))
camera.PushFilter(sens.ChFilterRGBA8Access())


manager.AddSensor(camera)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Mesh with Orbiting Camera")
vis.Initialize()
vis.AddLogo()
vis.AddCamera(chrono.ChVector3d(0, 2, 4))
vis.AddTypicalLights()
vis.AddSkyBox()


while vis.Run():
    
    orbit_angle = orbit_speed * sim_time
    cam_x = orbit_radius * math.cos(orbit_angle)
    cam_z = orbit_radius * math.sin(orbit_angle)
    camera.SetOffsetPose(chrono.ChFramed(
        chrono.ChVector3d(cam_x, orbit_height, cam_z),
        chrono.Q_from_AngAxis(orbit_angle + math.pi, chrono.ChVector3d(0, 1, 0))
    ))

    
    vis.BeginScene()
    vis.Render()
    manager.Update()
    sys.DoStepDynamics(time_step)
    vis.EndScene()
    
    
    if camera.GetMostRecentRGBA8Buffer():
        rgba8_buffer = camera.GetMostRecentRGBA8Buffer()
        rgba8_data = rgba8_buffer.GetRGBA8Data()
        print(f"Time: {sim_time:.2f}s | "
              f"Cam Pos: ({cam_x:.2f}, {orbit_height:.2f}, {cam_z:.2f}) | "
              f"First Pixel: {rgba8_data[0:3]}")
    
    sim_time += time_step