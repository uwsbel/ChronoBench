import os
import math
import numpy as np
import pychrono as chrono
import pychrono.sensor as chsensor
import pychrono irrlicht as chronoirr


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "chrono_data"))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Lidar Mesh Simulation")
vis.Initialize()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0, 3, 10))


mesh_path = os.path.join(os.path.dirname(__file__), "mesh.obj")
mesh = chrono.ChTriangleMeshShape()
mesh.SetFilename(mesh_path)
mesh.SetScale(1.0)
mesh.SetName("mesh")

body = chrono.ChBodyEasyConvexHull(mesh.GetPoints(), True, True)
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetBodyFixed(True)
body.AddVisualShape(mesh)
sys.Add(body)


sensor_manager = chsensor.ChSensorManager(sys)
lidar = chsensor.ChLidar()
lidar.SetPosition(chrono.ChVector3d(2, 0, 2))  
lidar.SetRotation(chrono.ChQuaternionD(1, 0, 0, 0))
lidar.SetRange(50.0)
lidar.SetHorizontalAngleRange(-1.5708, 1.5708)  
lidar.SetVerticalAngleRange(-0.7854, 0.7854)     
lidar.SetNoise(0.01)
lidar.SetGridResolution(0.1)
lidar.SetGridDimensions(100, 100)
lidar.SetPointCloudDimensions(1024, 256)
lidar.SetSaveData(True)
lidar.SetSavePath(os.path.join(os.path.dirname(__file__), "lidar_data"))
lidar.SetSaveFormat(chsensor.ChSensor.E_SAVEFORMAT_PLY)
lidar.SetVisualize(True)
lidar.SetPointColor(chrono.ChColor(1, 0, 0))
lidar.SetGridColor(chrono.ChColor(0, 1, 0))
sensor_manager.AddSensor(lidar)


grid = chrono.ChBody()
grid.SetPos(chrono.ChVector3d(0, 0, 0))
grid.SetBodyFixed(True)
grid.GetVisualShape(0).SetDrawMode(chrono.ChVisualization DrawMode::CH_DRAW_MODE_WIREFRAME)
sys.Add(grid)


simulation_time = 10.0
step_size = 0.01
lidar_radius = 5.0
lidar_angle = 0.0


while vis.Run() and sys.GetChTime() < simulation_time:
    
    lidar_angle += 0.01
    lidar.SetPosition(chrono.ChVector3d(
        lidar_radius * math.cos(lidar_angle),
        lidar_radius * math.sin(lidar_angle),
        2
    ))
    
    
    sensor_manager.Update()
    
    
    sys.DoStepDynamics(step_size)
    
    
    lidar_data = lidar.GetGridData()
    print("Lidar buffer data at time", sys.GetChTime())
    print("Min distance:", lidar_data.min())
    print("Max distance:", lidar_data.max())
    print("------------------------")
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()


vis.Close()