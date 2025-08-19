import pychrono as chrono
import pychrono.sensor as sens
import numpy as np
import math


chrono.SetChairsInertiaAutocalc(True)
chrono.SetChronoDataPath("chrono_data/")


mysystem = chrono.ChSystemNSC()


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh("chrono_data/models/tractor.obj")


body = chrono.ChBody()
body.SetPos(chrono.ChVector3d(0, 0, 0))
body.SetFixed(True)
body.AddAsset(mesh)
mysystem.Add(body)


camera = sens.ChCameraSensor(body, 600, chrono.ChVector3d(0, 0, 0), 0, 0)
camera.SetImageSize(1280, 720)
camera.SetFPS(60)
camera.SetName("Camera")
camera.SetUpdateRate(1)


camera_noise = sens.ChCameraNoiseModelType_GAUSSIAN
camera_noise_par = sens.ChNoiseParametersSensor()
camera_noise_par.SetAmp(0.01)
camera_noise_par.SetLen(0.01)
camera_noise_par.SetType(camera_noise)
camera.SetNoise(camera_noise_par)


manager = sens.ChSensorManager(mysystem)
manager.AddSensor(camera)


def callback(sensor, data):
    print("Camera buffer data:")
    print(data)


camera.SetOnSensorCallback(callback)


manager.Initialize()


vis_camera = chrono.ChCamera()
vis_camera.SetPos(chrono.ChVector3d(0, 0, 0))
vis_camera.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))


while True:
    mysystem.DoStepDynamics(1 / 60)
    manager.Update()
    vis_camera.SetPos(chrono.ChVector3d(5 * math.cos(chrono.GetChTime()), 5 * math.sin(chrono.GetChTime()), 2))
    vis_camera.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))
    vis_camera.SetRot(vis_camera.GetRot() * chrono.ChQuaterniond(math.cos(chrono.GetChTime() / 10), 0, 0, math.sin(chrono.GetChTime() / 10)))
    manager.WriteSimulation(chrono.GetChTime())