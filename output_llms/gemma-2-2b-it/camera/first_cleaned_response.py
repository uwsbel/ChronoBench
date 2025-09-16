import pychrono as chrono
import pychrono.visualization as vis
import pychrono.core as core
import pychrono.sensor as sensor
import numpy as np


chrono.init()


world = chrono.World()


mesh_path = "mesh.obj"  
mesh = chrono.Mesh.Load(mesh_path)


body = chrono.Body(mesh)
body.SetMass(1.0)  
body.SetInertiaTensor(chrono.InertiaTensor(mesh.vertices, mesh.faces))


camera_sensor = sensor.Sensor(body)


camera_sensor.SetFocalLength(100)
camera_sensor.SetAperture(1)
camera_sensor.SetNoiseFilter(sensor.NoiseFilter.Gaussian(sigma=0.1))


camera_sensor.SetPosition(chrono.Vec3(0, 0, 0))
camera_sensor.SetOrientation(chrono.Quaternion(0, 0, 0, 1))


sensor_manager = sensor.SensorManager(camera_sensor)


chrono.Simulate(world, 1000)


for i in range(100):
    
    camera_buffer = camera_sensor.GetBuffer()

    
    print(f"Camera Buffer at step {i}: {camera_buffer}")


chrono.Finalize()