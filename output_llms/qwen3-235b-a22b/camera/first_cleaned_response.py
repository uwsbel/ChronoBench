import pychrono as chrono
import pychrono.sensor as sens
import math




system = chrono.ChSystemNSC()
system.SetGravity(chrono.ChVectorD(0, 0, 0))




mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj", False, True)  


mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetBodyFixed(True)
system.Add(mesh_body)





update_rate = 30  
resolution = (640, 480)
fov_deg = 60  


initial_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.QUNIT)

camera = sens.ChCameraSensor(
    mesh_body, update_rate, initial_pose,
    resolution[0], resolution[1], fov_deg
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0.0)


camera.PushFilter(sens.ChFilterRGBA8Access())  
camera.PushFilter(sens.ChFilterVisualize(resolution[0], resolution[1], "Camera Feed"))  
camera.PushFilter(sens.ChFilterImageNoiseGaussian(0.01))  


manager = sens.ChSensorManager(system)
manager.AddSensor(camera)




vis = chrono.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Mesh with Orbiting Camera")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 0, 5), chrono.ChVectorD(0, 0, 0))




orbit_radius = 2.0
orbit_height = 1.0
angular_velocity = 0.1  
t = 0
time_step = 0.01

while vis.Run():
    
    angle = angular_velocity * t
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height
    cam_pos = chrono.ChVectorD(x, y, z)

    
    target = chrono.ChVectorD(0, 0, 0)
    dir_to_target = (target - cam_pos).GetNormalized()
    quat = chrono.ChQuaternionD().FromTwoVectors(
        chrono.ChVectorD(0, 0, -1),  
        dir_to_target
    )

    
    camera.SetOffsetPose(chrono.ChFrameD(cam_pos, quat))

    
    system.DoStepDynamics(time_step)
    manager.Update()

    
    buffer = camera.GetMostRecentRGBA8Buffer()
    if buffer.HasData():
        print(f"Time: {t:.2f} s, Buffer timestamp: {buffer.TimeStamp():.4f} s")

        
        width = buffer.Width()
        height = buffer.Height()
        if width > 0 and height > 0:
            center_pixel_index = (height // 2) * width + (width // 2)
            pixel = buffer.Data()[center_pixel_index]
            print(f"Center pixel (RGBA): {pixel.R}, {pixel.G}, {pixel.B}, {pixel.A}")

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    t += time_step