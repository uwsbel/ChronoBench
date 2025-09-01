import pychrono as chrono
import pychrono.visualization as vis
import pychrono.sensor as sensor
import pychrono.sensor.noise as noise
import pychrono.sensor.visualization as vis_sensor


chrono.init()


world = chrono.World()


mesh_path = "your_mesh.obj"  
mesh = chrono.Mesh.from_file(mesh_path)


mesh_body = chrono.Body(mesh)
mesh_body.set_fixed()
world.add_body(mesh_body)


camera_sensor = sensor.CameraSensor(mesh_body)


camera_sensor.add_noise_filter(noise.GaussianNoiseFilter(sigma=0.1))
camera_sensor.add_visualization(vis.ImageBuffer(width=640, height=480))


camera_sensor.set_position(chrono.Vector3(0, 0, 10))
camera_sensor.set_orientation(chrono.Quaternion(0, 0, 0, 1))


sensor_manager = sensor.SensorManager(camera_sensor)


world.run(1000)


for step in range(1000):
    
    buffer_data = camera_sensor.get_buffer_data()

    
    print(f"Step {step}: {buffer_data}")


world.destroy()