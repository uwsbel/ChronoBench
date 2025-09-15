import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as irr
import math
import os

def main():
    
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    system = chrono.ChSystemNSC()
    system.SetGravity(chrono.ChVector3d(0, 0, 0))  

    
    mesh_body = chrono.ChBody()
    mesh_body.SetFixed(True)
    
    
    obj_path = chrono.GetChronoDataFile("models/cube.obj")
    mesh = chrono.ChVisualShapeModelFile(obj_path)
    mesh_body.AddVisualShape(mesh)
    system.Add(mesh_body)

    
    manager = sens.ChSensorManager(system)
    manager.scene.AddPointLight(chrono.ChVector3d(2, 2, 2), chrono.ChColor(1, 1, 1), 1000)

    
    lens_model = sens.PinholeLens()
    lens_model.SetFocalLength(35)
    lens_model.SetExposureTime(0.02)
    
    
    noise_model = sens.NoiseNormalDist(0, 0.1)  
    filter_model = sens.PixelIntensityFilter(0.1, 1.0)  

    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(0, 0, 0), chrono.ChQuaterniond(1, 0, 0, 0))
    camera = sens.ChCameraSensor(
        mesh_body,              
        30,                     
        offset_pose,            
        1920,                   
        1080,                   
        math.radians(70),       
        lens_model              
    )
    camera.SetName("Orbiting Camera")
    camera.SetLag(0)
    camera.SetCollectionWindow(0.02)
    
    
    camera.PushFilter(filter_model)
    camera.PushNoiseModel(noise_model)
    
    
    camera.PushFilter(sens.FilterVisualize(1920, 1080, "Camera Output"))
    manager.AddSensor(camera)

    
    vis = irr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle("Mesh Camera Simulation")
    vis.Initialize()
    vis.AddLogo()
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(2, 2, 2), chrono.ChVector3d(0, 0, 0))
    vis.AddTypicalLights()

    
    orbit_radius = 3.0
    orbit_height = 1.0
    angular_speed = 0.1  
    time_step = 0.01
    total_time = 10.0

    
    time = 0
    while time < total_time:
        
        cam_x = orbit_radius * math.cos(angular_speed * time)
        cam_y = orbit_radius * math.sin(angular_speed * time)
        camera.SetOffsetPose(chrono.ChFramed(
            chrono.ChVector3d(cam_x, cam_y, orbit_height),
            chrono.QuatFromAngleZ(angular_speed * time + math.pi)
        ))

        
        manager.Update()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        system.DoStepDynamics(time_step)

        
        print(f"Time: {time:.2f}s | Camera Pos: ({cam_x:.2f}, {cam_y:.2f}, {orbit_height:.2f})")
        print(f"Camera Buffer: {camera.GetMostRecentBuffer()}")

        time += time_step

if __name__ == "__main__":
    main()