import pychrono as chrono
import pychrono.sensor as sens
import pychrono.irr as irr
import math


system = chrono.ChSystemNSC()
system.Set_Gravity(chrono.ChVectorD(0, 0, 0))  


mesh_body = chrono.ChBodyFixed()
mesh_body.SetName("MeshBody")
system.Add(mesh_body)


mesh_shape = chrono.ChTriangleMeshConnected()
mesh_shape.LoadWavefrontMesh("mesh.obj", False, False)
mesh_visual = chrono.ChVisualShapeTriangleMesh(mesh_shape, False, False)
mesh_body.AddAsset(mesh_visual)


sensor_manager = sens.ChSensorManager(system)


camera = sens.ChCameraSensor(
    mesh_body,  
    30,  
    chrono.ChFrameD(chrono.ChVectorD(2, 2, 2)),  
    640, 480  
)
camera.SetName("Camera Sensor")
camera.SetLag(0)
camera.SetCollectionWindow(1 / 30)


camera.PushFilter(sens.ChFilterCameraNoise())
camera.PushFilter(sens.ChFilterAccessBuffer(sens.ChFilterBufferType.RGBA))
camera.PushFilter(sens.ChFilterVisualize(640, 480))

sensor_manager.AddSensor(camera)


application = irr.ChIrrApp(system, "PyChrono Simulation", irr.dimension2du(800, 600))
application.AddTypicalCamera(chronoirr.vector3df(2, 2, 2))
application.AddTypicalLights()
application.SetShowInfos(True)
application.SetTargetBody(mesh_body)
application.SetTimestep(0.001)  


orbit_radius = 2
theta = 0
angular_velocity = 0.05  
time_step = 0.001
simulation_time = 10.0


time = 0
while time < simulation_time:
    
    pos_x = orbit_radius * math.cos(theta)
    pos_z = orbit_radius * math.sin(theta)
    pos = chrono.ChVectorD(pos_x, 0, pos_z)
    
    
    dir = -pos
    dir.Normalize()
    
    
    rot = chrono.ChQuaternionD()
    rot.SetFromDirs(chrono.ChVectorD(0, 0, -1), dir)
    
    camera.SetOffsetFrame(chrono.ChFrameD(pos, rot))
    
    
    system.DoStepDynamics(time_step)
    sensor_manager.Update()
    
    
    buffer = camera.GetMostRecentBuffer()
    if buffer:
        print(f"Time: {time:.3f}")
        data = buffer.GetRGBAidata()
        print(f"First 10 pixel values: {data[:10]}")
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    time += time_step
    theta += angular_velocity * time_step