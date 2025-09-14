```python
import chrono.core as chrono
import chrono.irrlicht as irrlicht
import chrono.sensor as sens
import math
import numpy as np


system = chrono.ChSystemNSC()


mesh_body = chrono.ChBody()
mesh_body.SetBodyFixed(True)


trimesh = chrono.ChTriangleMeshConnected()
trimesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"))  


visual_shape = chrono.ChTriangleMeshShape()
visual_shape.SetMesh(trimesh)
visual_shape.SetName("mesh_visual")
mesh_body.AddVisualShape(visual_shape)

system.Add(mesh_body)


manager = sens.ChSensorManager(system)
manager.scene.AddPointLight(chrono.ChVectorD(2, 2, 2), chrono.ChColor(1, 1, 1), 500)


offset_pose = chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), chrono.Q_ROTATE_Y_TO_Z)
cam = sens.ChCameraSensor(
    mesh_body,         
    30,                
    offset_pose,       
    640,               
    480,               
    chrono.CH_C_PI/3   
)


cam.SetName("Orbit Camera")
cam.SetLag(0.1)
cam.SetCollectionWindow(0.02)


noise_model = sens.ChNoiseNormalDist(0.0, 0.02)
cam.AddNoiseModel(noise_model)


cam.PushFilter(sens.ChFilterVisualize(640, 480, "Camera View"))


acc_filter = sens.ChFilterAccess()
cam.PushFilter(acc_filter)


manager.AddSensor(cam)


vis = irrlicht.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Camera Orbit Simulation")
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile("logo_pychrono_alpha.png"))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(3, 3, 3))
vis.AddTypicalLights()


radius = 3.0
angle = 0.0
angular_speed = 0.02