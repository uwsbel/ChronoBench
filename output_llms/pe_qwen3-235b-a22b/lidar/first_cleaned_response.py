import os
import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens


sys = chrono.ChSystemNSC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "data"))


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/mesh.obj"))  
mesh_body = chrono.ChBody()
mesh_body.SetMesh(mesh)
mesh_body.SetPos(chrono.ChVector3d(0, 0, 0))
mesh_body.SetFixed(True)  
sys.Add(mesh_body)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Orbiting Mesh Simulation")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


manager = sens.ChSensorManager(sys)


orbit_radius = 5.0
mount = chrono.ChBody()
sys.Add(mount)
mount.SetPos(chrono.ChVector3d(orbit_radius, 0, 1.0))  
mount.SetMass(1.0)
mount.SetInertiaXX(chrono.ChVector3d(0.1, 0.1, 0.1))


lidar = sens.ChLidarSensor(
    mount,                    
    100,                    
    10,                     
    chrono.ChFrameD(),      
    360,                    
    40,                     
    100,                    
    sens.LidarBeamShape_RECTANGULAR,  
    0.01,                   
    0.01,                   
    sens.LidarReturnMode_STRONGEST    
)


lidar.PushFilter(sens.ChFilterNoiseLidar(0.01))  


lidar.PushFilter(sens.ChFilterPCfromDepth())


lidar.PushFilter(sens.ChFilterVisualizePointCloud(640, 480, 1, "Point Cloud"))


lidar.PushFilter(sens.ChFilterSavePointCloud("output/"))


manager.AddSensor(lidar)


omega = 0.5  
time_step = 0.01

while vis.Run():
    
    current_time = sys.GetChTime()
    theta = omega * current_time
    x = orbit_radius * math.cos(theta)
    y = orbit_radius * math.sin(theta)
    z = 1.0  
    mount.SetPos(chrono.ChVector3d(x, y, z))

    
    sys.DoStepDynamics(time_step)

    
    manager.Update()

    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    buffer = lidar.GetMostRecentBuffer()
    if buffer and buffer.HasData():
        point_cloud = buffer.GetPointCloud()
        print(f"Current lidar data (first point): {point_cloud[0] if len(point_cloud) > 0 else 'No points'}")