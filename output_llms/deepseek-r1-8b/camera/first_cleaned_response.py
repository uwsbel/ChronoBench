import pychrono as pychrono
import pychrono.visualizations as pyvis
import numpy as np


pychrono.init()


scene = pychrono.Scene()

mesh = pychrono.Mesh("data/triangular_mesh.obj")

body = pychrono.Body()
body.set_mass(0)  
body.set_name("Triangular Mesh")
scene.add_body(body, mesh)


sensor = pychrono.Sensors.CameraSensor(body)

sensor_manager = pychrono.Sensors.SensorManager(sensor)

noise_filter = pychrono.Sensors.NoiseFilters.GaussianNoiseFilter(sigma=0.1)
sensor_manager.add_filter(noise_filter)


def update_camera(sim, body, time):
    
    radius = 5.0  
    angle = time * 0.5  
    
    body_local_pos = body.get_local_position()
    body_local_pos.x = radius * pychrono.maths.sin(angle)
    body_local_pos.y = radius * pychrono.maths.cos(angle)
    body.set_local_position(body_local_pos)


sensor.set_resolution(256, 256)  
sensor.set_focal_length(512.0)  
sensor.set_depth_mode(pychrono.Sensors.CameraSensor.DepthMode.PHI)  


renderer = pyvis.Renderer()
renderer.set_camera(sensor)
renderer.set_antialiasing(16)  


simulation_loop = pychrono.SimulationLoop()
simulation_loop.set_timestep(0.1)  


simulation_loop.add_function(update_camera, body, 0.1)


simulation_loop.start()


while True:
    
    image = sensor.get_image()
    
    img_array = np.array(image)
    print("Camera Image Data:")
    print(img_array)
    
    pychrono.sleep(0.1)