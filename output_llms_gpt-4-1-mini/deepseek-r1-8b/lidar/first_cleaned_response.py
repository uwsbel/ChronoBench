import pychrono as pychrono
import numpy as np
import os


pychrono.init()





mesh = pychrono.Mesh("example.obj", pychrono.DCS)

mesh.set_name("Triangular Mesh")
mesh.set_position(pychrono.Vector3d(0, 0, 0))
mesh.set_size(pychrono.Vector3d(1, 1, 1))
mesh.set_color(pychrono.Color(1, 1, 1))


lidar = pychrono.LidarSensor()
lidar.set_name("Lidar Sensor")

lidar.set_mount_point(mesh)

lidar.set_noise_model(pychrono.LidarNoiseModel.Poisson(1e-6))
lidar.set_filter_type(pychrono.LidarFilterType.GaussianBlur(0.1))
lidar.set_max_distance(10.0)





viewer = pychrono.Viewer()

camera = pychrono.Camera()
camera.set_position(pychrono.Vector3d(5, 5, 5))
camera.set_lookat(pychrono.Vector3d(0, 0, 0))
camera.set_up(pychrono.Vector3d(0, 1, 0))
viewer.set_camera(camera)

viewer.set_antialiasing(True)

viewer.add_mesh(mesh)
viewer.add_sensor(lidar)


orbit_radius = 5.0
orbit_height = 1.0
orbit_speed = 1.0
lidar_path = pychrono.Path()
lidar_path.set_type(pychrono.PathType.Circle)
lidar_path.set_radius(orbit_radius)
lidar_path.set_height(orbit_height)
lidar_path.set_period(2 * np.pi / orbit_speed)


timestep = 0.01
simulation_steps = 1000

for step in range(simulation_steps):
    
    lidar_path.update(step * timestep)
    lidar.set_position(lidar_path.get_position())
    
    
    data = lidar.get_buffer()
    
    
    print(f"Lidar Data at step {step}:")
    print("Timestamp: ", data.timestamp)
    print("Points: ", data.points)
    print("Depths: ", data.depths)
    print("\n")
    
    
    filename = f"lidar_data_{step}.csv"
    np.savetxt(filename, data, delimiter=',')
    
    
    pychrono.simulate(timestep)
    

viewer.close()