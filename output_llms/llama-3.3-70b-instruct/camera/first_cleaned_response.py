import pychrono.core as chrono
import pychrono.sensor as sensor
import pychrono.irrlicht as chronoirr
import numpy as np


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(800, 600)
vis.AddSkyBox()
vis.AddTypicalLights()



mesh = chrono.ChTriangleMesh()
mesh.LoadWavefrontMesh("mesh.obj")


body = chrono.ChBodyEasyMesh(mesh, 1000, True, True)
body.SetPos(chrono.ChVectorD(0, 0, 0))
body.SetRot(chrono.Q_from_AngX(0))
sys.Add(body)


body.SetBodyFixed(True)


cam = sensor.ChCameraSensor(body)
cam.SetName("camera")
cam.SetImageSize(640, 480)
cam.SetFocalLength(30)
cam.SetPosition(chrono.ChVectorD(0, 0, 1))
cam.SetLookAt(chrono.ChVectorD(0, 0, 0))


cam.AddNoiseFilter(sensor.ChNoiseFilterGaussian(0.1))
cam.AddVisualization(sensor.ChVisualizationImage())


sman = sensor.ChSensorManager(sys)
sman.AddSensor(cam)


for i in range(100):
    angle = i * np.pi / 50
    cam.SetPosition(chrono.ChVectorD(np.cos(angle), 0, np.sin(angle)))
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    print(cam.GetImageBuffer())


vis.Run()