import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors


system = chrono.ChSystemMulticore()
system.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN)
system.SetMaxItersSolverSpeed(50)
system.SetTimestep(0.01)


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)
mesh_body.SetPos(chrono.ChVectorD(0, 0, 0))
system.Add(mesh_body)


mesh_asset = chrono.ChTriangleMeshShape()
mesh_asset.SetMesh(chrono.loadWavefrontMesh('path/to/mesh.obj', 1.0))
mesh_body.AddAsset(mesh_asset)


visualizer = chronoirr.ChVisualizer(system)
visualizer.SetWindowTitle('Mesh with Camera')
visualizer.SetWindowSize(1280, 1024)
visualizer.SetCameraPosition(chrono.ChVectorD(0, 0, 3))
visualizer.SetCameraRotation(chrono.ChQuaternionD(1, 0, 0, 0))
visualizer.AddTypicalLights()


renderPlugin = visualizer.GetSystem().GetRenderPlugin()
sensor_manager = sensors.ChSensorManager(system, renderPlugin)
sensor_manager.SetRender(True)


camera = sensors.ChCameraSensor()
camera.SetName('main_camera')
camera.SetImageResolution(chrono.ChVectorI(640, 480, 0))
camera.SetFov(60.0)
camera.SetPosition(chrono.ChVectorD(3, 0, 2))  
camera.SetLookAt(chrono.ChVectorD(0, 0, 0))    


filter = sensors.ChNoiseFilterGaussian()
filter.SetMean(0.0)
filter.SetSigma(0.01)
camera.AddFilter(filter)

sensor_manager.AddSensor(camera)


while visualizer.Run():
    system.DoStepDynamics()
    
    
    time = system.GetChTime()
    angle = time * 2 * chrono.CH_C_PI  
    radius = 3.0
    x = radius * chrono.cos(angle)
    y = radius * chrono.sin(angle)
    z = 2.0  
    camera.SetPosition(chrono.ChVectorD(x, y, z))
    camera.SetLookAt(chrono.ChVectorD(0, 0, 0))  
    
    
    sensor_manager.Update()
    rgb_buffer = camera.GetRgbBuffer()
    
    
    print(f"Frame {system.GetChTime():.2f}s: Buffer {rgb_buffer.GetWidth()}x{rgb_buffer.GetHeight()}")
    pixel = rgb_buffer.GetPixel(0, 0)
    print(f"First pixel RGB: ({pixel.r:.2f}, {pixel.g:.2f}, {pixel.b:.2f})")
    
    
    visualizer.BeginScene()
    visualizer.DrawAll()
    visualizer.DrawSensorManager(sensor_manager)
    visualizer.EndScene()