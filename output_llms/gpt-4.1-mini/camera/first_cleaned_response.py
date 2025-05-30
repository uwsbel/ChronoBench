import pychrono as chrono
import pychrono.sensor as sens
import os


chrono.SetChronoDataPath(chrono.GetChronoDataPath())  

sys = chrono.ChSystemNSC()




body = chrono.ChBody()
body.SetBodyFixed(True)
sys.Add(body)


mesh_path = chrono.GetChronoDataPath() + "sensor/mesh.obj"
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(mesh_path, True, True)


mesh_shape = chrono.ChTriangleMeshShape()
mesh_shape.SetMesh(mesh)
mesh_shape.SetMutable(False)
body.AddVisualShape(mesh_shape)




sensor_manager = sens.ChSensorManager(sys)
sensor_manager.SetVerbose(True)
sensor_manager.SetWindowSize(800, 600)
sensor_manager.SetProcessingThreads(4)


cam = sens.ChCameraSensor(body)
cam.SetName("Camera Sensor")


cam.SetUpdateRate(50)
cam.SetResolution(800, 600)
cam.SetFOV(1.0)  


cam_pose = chrono.ChFrameD()
cam_pose.SetPos(chrono.ChVectorD(1, 0, 0.5))  
cam.SetOffsetPose(cam_pose)


cam.AddFilter(sens.ChFilterRGBDNoise(0.005))  
cam.AddFilter(sens.ChFilterVisualize(800, 600, "Camera View"))
cam.AddFilter(sens.ChFilterSave("camera_image.png"))  

sensor_manager.AddSensor(cam)


timestep = 0.01
sim_time = 5  


import math

radius = 1.5
height = 0.5
angular_speed = 0.4 * 2 * math.pi  

steps = int(sim_time / timestep)

for step in range(steps):
    time = step * timestep

    
    angle = angular_speed * time
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z = height

    cam.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(x, y, z), chrono.Q_from_AngAxis(-angle, chrono.ChVectorD(0, 0, 1))))

    
    sys.DoStepDynamics(timestep)
    sensor_manager.Update()

    
    rgb_buffer = cam.GetMostRecentRGB()
    
    width = cam.GetWidth()
    height = cam.GetHeight()

    
    if rgb_buffer is not None and len(rgb_buffer) >= 3:
        pix_r = rgb_buffer[0]
        pix_g = rgb_buffer[1]
        pix_b = rgb_buffer[2]
        print(f"Time: {time:.2f}s, Camera RGB first pixel: R={pix_r} G={pix_g} B={pix_b}")
    else:
        print(f"Time: {time:.2f}s, No RGB data available")

print("Simulation finished.")