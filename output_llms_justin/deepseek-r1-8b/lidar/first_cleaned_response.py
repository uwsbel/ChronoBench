import pychrono as pychrono
import numpy as np


pychrono.init()


scene = pychrono.Scene()
scene.set_ground_plane("horizontal", 0, 0, 0)


mesh = pychrono.SceneMesh("sphere.obj")

scene.add_mesh(mesh)

body = pychrono.Body("body")
body.set_mass(1)
body.set_position([0, 0, 5])
body.set_constraints(pychrono.BodyConstraintsfixed_position())


lidar_sensor = pychrono.SensorLidar(body, "lidar")
lidar_sensor.set_update_frequency(10)
lidar_sensor.set_noise_level(0.1)
lidar_sensor.add_visualization("lidar_points", color=(0, 0, 1), size=0.1)


orbit_radius = 5  
orbit_speed = 1  
lidar_height = 1  


lidar_position = np.array([orbit_radius, 0, lidar_height])


def update_lidar_position():
    global step, lidar_position
    step += 1
    angle = step * 2 * np.pi / 360  
    lidar_position = np.array([
        orbit_radius * np.cos(angle),
        orbit_radius * np.sin(angle),
        lidar_height
    ])


mesh_visualization = pychrono.VisualizationMesh(
    mesh,
    color=(1, 0, 0),
    wireframe=True
)
scene.add_visualization(mesh_visualization)


scene.add_sensor(lidar_sensor)


simulation = pychrono.Simulation(scene)
simulation.set_ground_friction(0.5)
simulation.set_gravity([0, 0, -9.81])


lidar_buffer = pychrono.Buffer()
lidar_sensor.set_save_to_file(lidar_buffer, "lidar_data", step=5)


lidar_visualization = pychrono.VisualizationPlot(
    lidar_buffer,
    x_axis="distance",
    y_axis="intensity",
    z_axis="height",
    show_legend=True
)
simulation.add_visualization(lidar_visualization)


simulation.start()


while True:
    simulation.step()
    update_lidar_position()
    data = lidar_buffer.get_data()
    print(f"Step {step}: Lidar Data - Distance: {data[0]} | Intensity: {data[1]} | Height: {data[2]}")
    if step % 5 == 0:
        lidar_buffer.save_to_file()
    if step % 1000 == 0:
        break