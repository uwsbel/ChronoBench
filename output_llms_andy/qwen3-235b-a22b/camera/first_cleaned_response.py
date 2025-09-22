import pychrono as chrono
import pychrono.sensor as sens
import math


chrono.SetChronoDataPath("PYCHRONO_DATADIR")  
system = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("mesh.obj", False, True)  


body = chrono.ChBody()
body.SetBodyFixed(True)
body.GetAssets().push_back(mesh)
system.Add(body)


manager = sens.ChSensorManager(system)


camera = sens.ChCameraSensor(
    body,  
    60,    
    chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.ChQuaternionD(1, 0, 0, 0)),  
    640,   
    480,   
    60,    
    sens.LENS_MODEL_PINHOLE  
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(0.01)


camera.AddFilter(sens.ChFilterNoiseGaussian(0.0, 0.02))


camera.AddFilter(sens.ChFilterVisualize(640, 480, "Camera View"))


manager.AddSensor(camera)


step_size = 0.01
orbit_radius = 5.0
orbit_height = 2.0
orbit_speed = 0.5  


for i in range(1000):
    time = i * step_size
    angle = orbit_speed * time

    
    x = orbit_radius * math.cos(angle)
    y = orbit_radius * math.sin(angle)
    z = orbit_height

    
    dir_vec = chrono.ChVectorD(-x, -y, -z).GetNormalized()

    
    from_vec = chrono.ChVectorD(0, 0, -1)  
    rot_quat = chrono.ChQuaternionD().GetQuaternionBetween(from_vec, dir_vec)

    
    camera.SetOffsetPose(chrono.ChFrameD(chrono.ChVectorD(x, y, z), rot_quat))

    
    system.DoStepDynamics(step_size)

    
    manager.Update()

    
    buffer = camera.GetMostRecentBuffer()
    if buffer:
        print(f"Step {i}, Camera buffer timestamp: {buffer.GetTimestamp():.4f}s")

    
    print(f"Camera position: ({x:.2f}, {y:.2f}, {z:.2f})")