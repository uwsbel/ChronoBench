import pychrono.core as chrono
import pychrono.sensor as sens
import pychrono.irrlicht as chronoirr
import math



def main():
    

    
    
    
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0,0,-9.81))

    
    
    
    
    mesh = chrono.ChTriangleMeshConnected()
    mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/lidar_target.obj'), 1, 0.1)
    
    bodyA = chrono.ChBody()
    bodyA.SetPos(chrono.ChVector3d(0, 0, 0))
    bodyA.SetFixed(True)
    bodyA.GetVisualShape(0).SetMesh(mesh)
    bodyA.EnableCollision(0)
    sys.Add(bodyA)

    
    
    
    
    sens_manager = sens.ChSensorManager(sys)
    
    sens_manager.AddPointLight(chrono.ChVector3f(2,2.2,2000), chrono.ChColor(2,2.2,2))
    
    offset_pose = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    lidar = sens.ChLidarSensor(bodyA, offset_pose, chrono.LidarSensorDataTypes_MESH,  
                               360,  
                               0, 0,  
                               10,  
                               0.01, 0.01,  
                               5,  
                               0.005)  
    
    lidar.PushFilter(sens.ChFilterLidarNoise(lidar, 0.01))
    lidar.PushFilter(sens.ChFilterVisualize(1000))
    lidar.PushFilter(sens.ChFilterSave(1000, chrono.GetChronoDataFile("lidar_data/"), "out"))
    
    sens_manager.AddSensor(lidar)
    
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(sys)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle('Lidar Demo')
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddSkyBox()
    vis.AddCamera(chrono.ChVector3d(1.5,2.5,0), chrono.ChVector3d(0,0,0))
    vis.AddTypicalLights()
    
    time = 0
    time_step = 5e-3
    time_end = 15
    lidar_offset = chrono.ChFramed(chrono.ChVector3d(-8, 0, 2), chrono.QuatFromAngleAxis(0, chrono.ChVector3d(0, 1, 0)))
    orbit_radius = 10
    orbit_rate = 0.2
    orbit_time = 0
    while time < time_end:
        time = sys.GetChTime()
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        
        orbit_angle = orbit_time * orbit_rate
        lidar_offset.SetPos(chrono.ChVector3d(-8 * math.cos(orbit_angle), -8 * math.sin(orbit_angle), 2))
        lidar.SetOffsetPose(lidar_offset)

        
        sens_manager.Update()
        sys.DoStepDynamics(time_step)

        orbit_time += time_step

    return 0






main()