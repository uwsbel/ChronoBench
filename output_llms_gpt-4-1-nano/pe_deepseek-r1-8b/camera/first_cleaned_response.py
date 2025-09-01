import os
import math
import numpy as np
import chrono as chrono
from chronoirr import ChVisualSystemIrrlicht
from pychrono import ChSensorManager


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('path/to/data')  


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddSkyBox()


mesh_shape = chrono.ChVisualShape()
mesh_shape.LoadFromWavefront('triangular_mesh.obj')
mesh_shape.SetColor(chrono.ChColor(1, 0, 0))  
sys.AddVisualShape(mesh_shape)


camera_pos = chrono.ChVector3d(5, 5, 5)
camera_rot = chrono.Quat()
camera_rot.SetFromAxisAng(chrono.CH_PI/2, 1, 0, 0)  


sys.AddCamera(camera_pos, camera_rot)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sys.GetBody(0), sys.GetBody(1), chrono.ChCoordsysD(camera_pos, camera_rot))


motor.SetSpeedFunction(chrono.ChFunction_Sine(0.05, 1.0, True))  


sensor_manager = ChSensorManager(sys)
sensor_manager.AddSensor(0, 'camera', 0)  
sensor_manager.Enable(chrono.ChSensorMode.Off)  


noise_filter = chrono.ChSensorNoiseFilter()
noise_filter.SetNoiseLevel(0.1)
noise_filter.SetFilterSize(5)
sensor_manager.SetNoiseFilter(noise_filter)


sensor_manager.Enable(chrono.ChSensorMode.On)


def print_camera_data():
    pos = sys.GetBody(0).GetPos()
    rot = sys.GetBody(0).GetRot()
    img = sensor_manager.GetImage()
    
    print(f"Camera Position: {pos}")
    print(f"Camera Rotation: {rot}")
    print(f"Image Data: {img}")
    print("----------------------------")


while True:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    motor.Update()
    sys.DoStepDynamics(0.01)
    print_camera_data()

import os
import math
import numpy as np
import chrono as chrono
from chronoirr import ChVisualSystemIrrlicht
from pychrono import ChSensorManager


sys = chrono.ChSystemSMC()
chrono.SetChronoDataPath('path/to/data')  


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.Initialize()
vis.AddSkyBox()


mesh_shape = chrono.ChVisualShape()
mesh_shape.LoadFromWavefront('triangular_mesh.obj')
mesh_shape.SetColor(chrono.ChColor(1, 0, 0))  
sys.AddVisualShape(mesh_shape)


camera_pos = chrono.ChVector3d(5, 5, 5)
camera_rot = chrono.Quat()
camera_rot.SetFromAxisAng(chrono.CH_PI/2, 1, 0, 0)  


sys.AddCamera(camera_pos, camera_rot)


motor = chrono.ChLinkMotorRotationSpeed()
motor.Initialize(sys.GetBody(0), sys.GetBody(1), chrono.ChCoordsysD(camera_pos, camera_rot))


motor.SetSpeedFunction(chrono.ChFunction_Sine(0.05, 1.0, True))  


sensor_manager = ChSensorManager(sys)
sensor_manager.AddSensor(0, 'camera', 0)  
sensor_manager.Enable(chrono.ChSensorMode.Off)  


noise_filter = chrono.ChSensorNoiseFilter()
noise_filter.SetNoiseLevel(0.1)
noise_filter.SetFilterSize(5)
sensor_manager.SetNoiseFilter(noise_filter)


sensor_manager.Enable(chrono.ChSensorMode.On)


def print_camera_data():
    pos = sys.GetBody(0).GetPos()
    rot = sys.GetBody(0).GetRot()
    img = sensor_manager.GetImage()
    
    print(f"Camera Position: {pos}")
    print(f"Camera Rotation: {rot}")
    print(f"Image Data: {img}")
    print("----------------------------")


while True:
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    motor.Update()
    sys.DoStepDynamics(0.01)
    print_camera_data()