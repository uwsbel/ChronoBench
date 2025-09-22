import chrono as ch
import numpy as np
import matplotlib.pyplot as plt


ch.initialize()


world = ch.ChWorld()


body = ch.ChBody()
body.LoadWavefrontMesh("mesh.obj")
body.SetBodyFixed(True)
world.Add(body)


camera = ch.ChCamera()
camera.SetBody(body)
camera.SetSensor(True)
camera.SetSensorMode(ch.ChSensorMode.CAMERA)
camera.SetSensorType(ch.ChSensorType.CAMERA)
camera.SetSensorResolution(640, 480)
camera.SetSensorFOV(60)


sensor_manager = ch.ChSensorManager()
sensor_manager.AddSensor(camera)


noise_filter = ch.ChNoiseFilter()
noise_filter.SetSigma(0.1)
noise_filter.SetMu(0.5)
camera.SetSensorNoiseFilter(noise_filter)


visualization = ch.ChVisualization()
visualization.SetMode(ch.ChVisualizationMode.CAMERA)
camera.SetSensorVisualization(visualization)


camera.SetPosition(1, 1, 1)


world.SetGravity(0, 0, -9.81)
world.SetTimeStep(0.01)
world.SetMaxStep(0.1)
world.SetIterations(10)


for i in range(1000):
    world.DoStep()
    camera_buffer = camera.GetCameraBuffer()
    print(camera_buffer)
    
    camera.SetPosition(np.cos(i * 0.01), np.sin(i * 0.01), 1)


ch.finalize()