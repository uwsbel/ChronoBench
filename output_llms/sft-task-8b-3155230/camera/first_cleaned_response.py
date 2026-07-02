import pychrono as chrono
from pychrono import vis
from pychrono.sensor import ChSensorManager, ChCameraSensor, ChNoiseGaussian, ChImageFloat

system = chrono.ChSystemNSC()
vis_system = vis.ChVisualSystemIrrlicht()
vis_system.AttachSystem(system)
vis_system.SetWindowSize(1024, 768)
vis_system.SetWindowTitle('PyChrono Mesh Camera Sensor')
vis_system.Initialize()
vis_system.AddSkyBox()
vis_system.AddCamera(chrono.ChVector3d(0, 0, 5), chrono.ChVector3d(0, 0, 0))


mesh = chrono.ChBodyEasyMesh()
mesh.SetFixed(True)
mesh.SetMass(0)
mesh.LoadWavefrontMesh('mesh.obj', True, True)
mesh.GetVisualShape(0).SetColor(chrono.ChColor(0.5, 0.5, 0.5))
system.AddBody(mesh)


camera_body = chrono.ChBody()
camera_body.SetFixed(True)
system.AddBody(camera_body)


sensor = ChCameraSensor(camera_body, 10)
sensor.SetUpdateRate(10)
sensor.SetImageResolution(640, 480)
noise = ChNoiseGaussian(0.01, 0.01)
sensor.AddNoiseFilter(noise)
sensor_manager = ChSensorManager()
sensor_manager.AddSensor(sensor)


time = 0.0
while vis_system.Run():
    time += 0.01
    
    radius = 5.0
    height = 2.0
    angle = time * 0.1
    camera_x = radius * chrono.cos(angle)
    camera_y = radius * chrono.sin(angle)
    camera_z = height
    camera_body.SetPos(chrono.ChVector3d(camera_x, camera_y, camera_z))
    
    mesh_pos = mesh.GetPos()
    direction = mesh_pos - camera_body.GetPos()
    direction.Normalize()
    from_to = chrono.ChQuaternionFromTo(chrono.ChVector3d(0, 0, -1), direction)
    camera_body.SetRot(from_to)
    
    system.DoStepDynamics(0.01)
    sensor_manager.Update()
    
    image = sensor.GetImage()
    print(f"Camera image size: {image.width}x{image.height}")
    
    vis_system.BeginScene()
    vis_system.Render()
    vis_system.EndScene()