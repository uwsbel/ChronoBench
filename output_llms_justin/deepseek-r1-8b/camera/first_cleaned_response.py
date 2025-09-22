import pychrono as pc
from pychrono import core, mesh, sensors, movement
import numpy as np


pc.init()


mesh_path = "triangle_mesh.obj"
mesh = pc.load_obj(mesh_path)


body = pc.create_fixed_body(mesh, mass=0)  


sensor_manager = pc.SensorManager(body)
sensor = pc.create_camera_sensor(sensor_manager, name="orbital_camera")


from pychrono.sensors import gaussian_filter
sensor.add_noise_filter(gaussian_filter)


sensor.sensor_properties.sensor_type = pc.CAMERA_TYPE PINHOLE
sensor.sensor_properties.resolution = 512  
sensor.sensor_properties.frame_rate = 30  


pivot_point = body.get_body_point("center")  
rotational_motion = pc.create_rotational_movement(pivot_point, body, name="camera_orbit")


rotational_motion.set_rotation_axis(pc.Vec3d(0, 0, 1))


rotational_motion.set_rotation_speed(2)  


time_step = 0.01


simulation = pc.Simulation(body, time_step, max_time=1000, gui=True)


def print_camera_data(frame):
    image = sensor.get_image()
    print(f"Frame: {frame}")
    print(f"Camera buffer data: {image}")
    print("\n")


simulation.add_event("camera_orbit", callback=print_camera_data, event_type=pc.SIM_EVENT_STEP)


simulation.run()