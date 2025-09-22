import chrono as ch
import numpy as np
import matplotlib.pyplot as plt


ch.initialize()


world = ch.ChWorld()


body = ch.ChBody()
body.SetBodyFixed(True)
body.SetBodyInertia(1, 1, 1, 0, 0, 0)
body.LoadObjectFromWavefront("mesh.obj")
world.Add(body)


camera = ch.ChCameraSensor()
camera.SetSensorType(ch.CT_CAMERA_SENSOR)
camera.SetSensorId(0)
camera.SetSensorWidth(640)
camera.SetSensorHeight(480)
camera.SetSensorFocalLength(50)
camera.SetSensorXsize(640)
camera.SetSensorYsize(480)
camera.SetSensorXcenter(320)
camera.SetSensorYcenter(240)


sensor_manager = ch.ChSensorManager()
sensor_manager.Add(camera)


noise_filter = ch.ChNoiseFilter()
noise_filter.SetFilterType(ch.CT_FILTER_GAUSSIAN)
noise_filter.SetFilterSigma(5)


camera.SetSensorFilter(noise_filter)


camera_visualization = ch.ChCameraVisualization()
camera_visualization.SetVisualizationType(ch.CT_VISUALIZATION_CAMERA)
camera_visualization.SetVisualizationId(0)
camera_visualization.SetVisualizationWidth(640)
camera_visualization.SetVisualizationHeight(480)
camera_visualization.SetVisualizationFocalLength(50)
camera_visualization.SetVisualizationXsize(640)
camera_visualization.SetVisualizationYsize(480)
camera_visualization.SetVisualizationXcenter(320)
camera_visualization.SetVisualizationYcenter(240)


sensor_manager.Add(camera_visualization)


camera.SetSensorPosition(1, 0, 0)
camera.SetSensorOrientation(0, 0, 0, 1)


for i in range(100):
    world.StepSimulation(0.01)
    camera_data = camera.GetSensorData()
    print(camera_data)
    camera.SetSensorPosition(np.cos(i * 0.1) * 2, 0, np.sin(i * 0.1) * 2)
    camera.SetSensorOrientation(0, 0, 0, 1)


ch.finalize()