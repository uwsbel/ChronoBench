import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sensors
import pychrono.postprocess as postprocess
import numpy as np


chrono.SetChronoDataPath('../data/')  


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('PyChrono Camera Sensor Simulation')
vis.Initialize()
vis.AddTypicalSky()
vis.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_small.png'))
vis.AddLightWithShadow(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(1, 1, 1), 1, 1, 50, 50)
vis.AddCamera(chrono.ChVectorD(2, 2, 2), chrono.ChVectorD(0, 0, 0))


mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('path/to/your/mesh.obj'))  
meshbody = chrono.ChBodyEasyBox(1, 1, 1, 1000)
meshbody.GetCollisionModel().ClearModel()
meshbody.GetCollisionModel().AddTriangleMesh(mesh, False, False)
meshbody.GetCollisionModel().BuildModel()
meshbody.SetPos(chrono.ChVectorD(0, 0, 0))
meshbody.SetFixed(True)
system.Add(meshbody)


sensor_manager = sensors.ChSensorManager(system)
camera_sensor = sensors.ChCameraSensor()
camera_sensor.SetName("CameraSensor")
camera_sensor.SetParent(meshbody)
camera_sensor.SetPosition(chrono.ChVectorD(2, 2, 2))
camera_sensor.SetOrientation(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1)))
camera_sensor.SetFov(chrono.ChFovAngle(chrono.CH_C_PI / 4))
camera_sensor.SetResolution(640, 480)
camera_sensor.SetNearPlane(0.1)
camera_sensor.SetFarPlane(100)


noise_filter = sensors.ChNoiseFilterGaussian(0.01, 0.01, 0.01)
camera_sensor.AddFilter(noise_filter)


visualization = sensors.ChVisualizationIrrlicht(camera_sensor, vis)
camera_sensor.AddVisualization(visualization)


sensor_manager.AddSensor(camera_sensor)


step_size = 0.01
total_time = 10.0
camera_radius = 2.0
camera_height = 2.0


for t in np.arange(0, total_time, step_size):
    
    angle = t * 2 * np.pi / total_time
    camera_pos = chrono.ChVectorD(camera_radius * np.cos(angle), camera_height, camera_radius * np.sin(angle))
    camera_sensor.SetPosition(camera_pos)
    camera_sensor.SetOrientation(chrono.Q_from_AngAxis(chrono.ChMatrix33D(1)))

    
    system.DoStepDynamics(step_size)

    
    camera_buffer = camera_sensor.GetDataBuffer()
    if camera_buffer:
        print(f"Time: {t:.2f} s, Camera Buffer Data: {camera_buffer}")


vis.Run()