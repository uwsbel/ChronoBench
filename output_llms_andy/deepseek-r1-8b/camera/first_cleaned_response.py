import pychrono as pc
import numpy as np


pc.init()


mesh = pc.load_obj("triangle.obj")


mesh.set_body_type(pc.BodyType.FIXED)


sensor = pc.SensorManager()
sensor_body = mesh.add_sensor(sensor)


sensor.camera_noise_level = 0.1  
sensor.contrastEnhancement = 1.2  
sensor.buffer_size = 512  


camera = pc.Camera()
camera.position = np.array([5, 5, 5])
camera.look_at = np.array([0, 0, 0])
camera.orbit_radius = 5  
camera.orbit_speed = 0.5  


sensor_body.add_sensor_camera(sensor, camera)


sim = pc.Simulation()
sim.set_time_step(0.1)  
sim.set_max_steps(1000)  


for step in range(sim.get_steps()):
    sim.step()
    sensor_data = sensor.get_image()
    print(f"Sensor Data at step {step}:")
    print(sensor_data)


sim.run()