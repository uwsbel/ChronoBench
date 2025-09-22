import os
import math
import numpy as np


import pychrono as chrono
from pychrono import fea
from pychrono.robot import *
from pychrono.sensors import *
from pychrono.visualization import chronoirr
from pychrono.materials import *
from pychrono.contact import *


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath(os.path.expanduser('~/.chrono/data'))  


mesh = fea.ChMesh()

mesh.Load('triangle.obj')


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetMass(100)
body.SetFixed(True)
sys.Add(body)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.Initialize()

mesh_shape = chrono.ChVisualShapeSphere(1.0)
body.AddVisualShape(mesh_shape)


sensor_manager = ChSensorManager(sys)
lidar = sensor_manager.CreateLidarSensor()
lidar.SetNoise(0.1)
lidar.SetSensorRange(10.0)
lidar.SetUpdateFrequency(100)
lidar.SetBody(body)
lidar.Initialize()


lidar_position = chrono.ChVector3d(0, 0, 0)
orbit_radius = 5.0
orbit_speed = 0.1

def UpdateLidarPosition():
    global lidar_position, orbit_radius, orbit_speed
    lidar_position.x = orbit_radius * math.sin(chrono.GetTime() * orbit_speed)
    lidar_position.y = orbit_radius * math.cos(chrono.GetTime() * orbit_speed)
    lidar_position.z = 0.0


lidar_data_buffer = []
lidar_data_filename = 'lidar_data.csv'
data_writer = None

def SaveLidarData():
    global lidar_data_buffer, lidar_data_filename, data_writer
    if not lidar_data_buffer:
        
        data_writer = open(lidar_data_filename, 'w')
        data_writer.write('x, y, z, distance, intensity\n')
    else:
        
        np.savetxt(lidar_data_filename, np.array(lidar_data_buffer), delimiter=',')
        lidar_data_buffer = []


while True:
    sys.DoStepDynamics(0.01)
    
    
    UpdateLidarPosition()
    
    
    point = chrono.ChVector3d(lidar_position.x, lidar_position.y, lidar_position.z)
    distance = 5.0  
    intensity = 100.0  
    lidar_data = [point.x, point.y, point.z, distance, intensity]
    lidar_data_buffer.append(lidar_data)
    
    
    SaveLidarData()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if os.path.exists(lidar_data_filename):
        break


if data_writer:
    data_writer.close()