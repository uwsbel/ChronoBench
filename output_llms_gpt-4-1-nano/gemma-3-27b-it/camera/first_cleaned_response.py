import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensor


chrono.SetChronoDataPath("./chrono_data")







sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)







mesh_file = "sphere.obj"  
mesh_path = os.path.join(chrono.GetChronoDataFile(""), mesh_file)


body = chrono.ChBody()
body.SetBodyFixed(True)  
body.SetCollide(True)
body.SetMass(1.0)
body.SetPos(chrono.ChVector3d(0, 0, 0))

mesh_shape = chrono.ChVisualShapeFile()
mesh_shape.SetFile(mesh_path)
body.AddVisualShape(mesh_shape)


sys.Add(body)







sensor_manager = sensor.ChSensorManager(sys)


camera = sensor.ChCameraSensor()
camera.SetBody(body)
camera.SetOffset(chrono.ChVector3d(0, 0, 2))  
camera.SetResolution(640, 480)
camera.SetFov(math.pi / 3.0)
camera.SetNear(0.1)
camera.SetFar(100.0)


noise_filter = sensor.ChNoiseGaussian()
noise_filter.SetMean(0.0)
noise_filter.SetStddev(0.01)
camera.AddNoiseFilter(noise_filter)


visualization = sensor.ChSensorVisualizationGrayscale()
camera.AddVisualization(visualization)


sensor_manager.AddSensor(camera)







vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Camera Sensor Simulation')
vis.Initialize()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()
vis.AddSkyBox()






time_step = 0.01

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    sys.DoStepDynamics(time_step)

    
    buffer = camera.GetBuffer()
    if buffer is not None:
        print(f"Camera buffer size: {len(buffer)}")
        
        
        if len(buffer) > 10:
            print(f"First 10 buffer values: {buffer[:10]}")
    else:
        print("Camera buffer is empty.")