import pychrono
import pychrono.core as pc
import pychrono.core.objects as o
import pychrono.core.sensors as sc
import pychrono.core.visuals as v
import pychrono.core.sensors.lidar as lidar
import numpy as np
import matplotlib.pyplot as plt


mesh_file = "triangle_mesh.obj"  
sensor_manager = pc.SensorManager()
lidar_sensor = lidar.LidarSensor(mesh_file)
simulation_time = 10  
orbit_radius = 10  
noise_level = 0.1 
visualization_interval = 100  
lidar_buffer_size = 1000 


try:
    pychrono.initialize()
except Exception as e:
    print(f"Error initializing PyChrono: {e}")
    exit()


try:
    mesh = o.Mesh(mesh_file)
except Exception as e:
    print(f"Error loading mesh: {e}")
    exit()



try:
    mesh.vertices = mesh.vertices.astype(float)
    mesh.faces = mesh.faces.astype(float)
    mesh.normals = mesh.normals.astype(float)
except Exception as e:
    print(f"Error setting up mesh: {e}")
    exit()


try:
    sensor_manager.add_sensor(lidar_sensor)
except Exception as e:
    print(f"Error adding sensor: {e}")
    exit()


try:
    for i in range(simulation_time):
        
        lidar_sensor.update_position(mesh.vertices[i])

        
        print(f"Simulation Step: {i}, Lidar Data: {lidar_sensor.data}")

        
        v.visualize(mesh, sensor_manager)

        
        if (i + 1) % visualization_interval == 0:
            print(f"Visualization Interval: {visualization_interval}")

except Exception as e:
    print(f"Simulation error: {e}")
    break


try:
    v.save_visualization(mesh, sensor_manager, visualization_interval)
    print("Visualization saved.")
except Exception as e:
    print(f"Error saving visualization: {e}")
    

finally:
    if 'mesh' in locals():
        mesh.close()
    if 'lidar_sensor' in locals():
        lidar_sensor.close()
    if 'sensor_manager' in locals():
        sensor_manager.close()
    pychrono.shutdown()