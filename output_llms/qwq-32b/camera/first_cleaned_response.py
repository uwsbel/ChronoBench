import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens
import math


system = chrono.ChSystemNSC()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetSolverMaxIterations(100)
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


visualizator = chronoirr.ChVisualizer()
visualizator.SetSystem(system)
visualizator.SetWindowTitle('Mesh and Camera Simulation')
visualizator.SetWindowSize(1280, 1024)
visualizator.SetSymbolsScale(0.01)
visualizator.SetShadows(True)
visualizator.SetLightIntensity(0.8)
visualizator.SetLightPosition(chrono.ChVectorD(0, 0, 10))
visualizator.SetBackgroundColor(chrono.ChColor(0.8, 0.8, 0.8))


sensor_manager = sens.ChSensorManager()
sensor_manager.SetSystem(system)


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)


mesh_shape = chrono.ChTriangleMeshShape()
if not mesh_shape.LoadWavefrontMesh('path/to/mesh.obj'):
    raise RuntimeError("Error loading mesh")
mesh_body.AddAsset(mesh_shape)
mesh_body.AddAsset(chrono.ChColorAsset(0.7, 0.7, 0.7))


camera_body = chrono.ChBody()
camera_body.SetBodyFixed(False)
system.Add(camera_body)
radius = 5.0
angular_velocity = 0.1  


sensor = sens.ChCameraSensor()
sensor.SetName('orbit_camera')
sensor.SetAttachBody(camera_body)
sensor.SetResolution(640, 480)
sensor.SetFov(90)  
sensor.SetNearPlane(0.1)
sensor.SetFarPlane(100)
sensor.AddNoiseFilter(sens.ChNoiseFilterGaussian(0.0, 0.1))  
sensor_manager.AddSensor(sensor)
sensor_manager.SetVisualization(True)


visualizator.Initialize()
visualizator.AddTypicalLights()
visualizator.AddCameraOrbitAndRotation()

step_size = 0.01
time_step = 0

while visualizator.Run():
    time = system.GetChTime()
    
    
    angle = angular_velocity * time
    x = radius * math.cos(angle)
    y = radius * math.sin(angle)
    z = 0.0
    
    camera_body.SetPos(chrono.ChVectorD(x, y, z))
    camera_body.SetPos_dt(chrono.ChVectorD(0, 0, 0))  
    camera_body.SetPos_dtdt(chrono.ChVectorD(0, 0, 0))  
    
    
    look_dir = chrono.ChVectorD(-x, -y, -z)
    up_dir = chrono.ChVectorD(0, 0, 1)
    frame = chrono.ChFrameD()
    frame.SetLookAt(camera_body.GetPos(), chrono.ChVectorD(0, 0, 0), up_dir)
    camera_body.SetFrame_REF_to_abs(frame)
    
    
    system.DoStepDynamics(step_size)
    
    
    sensor_manager.Update()
    image_data = sensor.GetImageData()
    print(f"Time {time:.2f}s - Resolution: {image_data.width()}x{image_data.height()}")
    
    
    visualizator.BeginScene()
    visualizator.DrawAll()
    visualizator.EndScene()
    
    if visualizator.GetDevice().getKeyPress(chrono.KEY_ESCAPE):
        break

visualizator.Close()