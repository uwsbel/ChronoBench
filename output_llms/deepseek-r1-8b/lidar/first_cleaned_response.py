import pychrono as pychrono
from pychrono.objects import *
from pychrono.sensors import *
from pychrono.render import *
import numpy as np


pychrono.init()


scene = pychrono.Scene()


obj = pychrono.objects.MeshObject("triangular_mesh.obj")
scene.add_object(obj)


obj.set_body_type("fixed")


lidar_sensor = pychrono.sensors.LidarSensor()
lidar_sensor.set_noise_level(0.1)  
lidar_sensor.set_parent(obj)  


orbit_radius = 5.0  
orbit_vertical = 5.0  
orbit_speed = 1.0  
orbit_period = 10.0  

lidar = pychrono.objects.Body()
lidar.set_body_type("dynamic")
lidar.set_mass(0.0)  
lidar.set_size(0.1, 0.1, 0.1)  
lidar.set_position(obj.get_position() + pychrono.Vec3(orbit_radius, 0, orbit_vertical))
lidar.set_velocity(pychrono.Vec3(orbit_speed, 0, 0))


orbit_constraint = pychrono.constraints.OrbitConstraint(lidar, obj)
scene.add_constraint(orbit_constraint)


camera = pychrono.objects.Camera()
camera.set_position(pychrono.Vec3(10, 10, 10))
camera.set_look_at(pychrono.Vec3(0, 0, 0))
camera.set_up(pychrono.Vec3(0, 1, 0))
renderer = pychrono.render.Renderer(camera)
renderer.setClearColor(0.5, 0.5, 0.5)


lidar_visual = pychrono.render.PointCloudVisualizer()
lidar_visual.set_source(lidar.get_source())
renderer.add(lidar_visual)


mesh_visual = pychrono.render.MeshVisualizer()
mesh_visual.set_source(obj.get_source())
renderer.add(mesh_visual)


simulation_time = pychrono.SimTime(0, 1.0)  
final_time = pychrono.SimTime(10, 1.0)    
dt = pychrono.SimTime(0.01, 1.0)        


lidar_buffer = []
current_time = 0.0


while simulation_time < final_time:
    
    scene.update()
    
    
    current_time += dt.get_value()
    angle = (current_time / orbit_period) * 360.0
    lidar.set_position(obj.get_position() + pychrono.Vec3(
        orbit_radius * np.cos(angle * np.pi / 180),
        0,
        orbit_vertical + orbit_radius * np.sin(angle * np.pi / 180)
    ))

    
    lidar_data = lidar.get_source().get_point_cloud()
    lidar_buffer.append(lidar_data.copy())
    
    
    print(f"Lidar Data at time {current_time:.3f}:")
    print("PointCloud Shape:", lidar_data.shape)
    print("First few points:", lidar_data[:3, :])
    
    
    simulation_time += dt


with open("lidar_data.npy", "wb") as f:
    np.save(f, np.array(lidar_buffer))


renderer.close()