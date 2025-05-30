import pychrono as pychrono
import pychrono.viewer.iranengine as iran
import numpy as np


pychrono.init()


scene = pychrono.Scene()


truss = []

truss_segments = [
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box"),
    pychrono.Mesh("box")
]


truss_positions = np.array([
    [0, 0, 0],
    [5, 0, 0],
    [5, 5, 0],
    [0, 5, 0],
    [2.5, 0, 0],
    [2.5, 5, 0],
    [3, 2.5, 0],
    [3, 2.5, 0]
])


for i in range(len(truss_segments)):
    truss_segments[i].set_position(truss_positions[i])


for i in range(len(truss_segments)):
    truss_segments[i].set_material(pychrono.MaterialFixed())


for seg in truss_segments:
    scene.add_object(seg)


rotating_bar = pychrono.Mesh("cylinder")
rotating_bar.set_radius(0.5)
rotating_bar.set_height(2)
rotating_bar.set_material(pychrono.MaterialFixed())


bar_position = np.array([2.5, 2.5, 0])
rotating_bar.set_position(bar_position)


scene.add_object(rotating_bar)


rotational_force = pychrono.Forces()
rotational_force.set_z(100)
rotational_force.set_torque(10)
rotating_bar.add_force(rotational_force)


fixed_gear = pychrono.Mesh("cylinder")
fixed_gear.set_radius(0.5)
fixed_gear.set_height(2)
fixed_gear.set_position(np.array([5, 0, 0]))
fixed_gear.set_material(pychrono.MaterialGold())
scene.add_object(fixed_gear)


moving_gear = pychrono.Mesh("cylinder")
moving_gear.set_radius(0.4)
moving_gear.set_height(2)
moving_gear.set_position(np.array([5, 3, 0]))
moving_gear.set_material(pychrono.MaterialGold())
scene.add_object(moving_gear)


radius = 0.5
teeth = 20


gear_motor = pychrono.Mesh("cylinder")
gear_motor.set_radius(0.5)
gear_motor.set_height(2)
gear_motor.set_position(np.array([5, 3, 0]))
gear_motor.set_material(pychrono.MaterialGold())


moving_gear.add_object(gear_motor)


motor_torque = 50
motor_force = 100


viewer = iran.IrrViewer(scene)
viewer.set_active_window()
viewer.set_hmd_mode(False)
viewer.show()


camera_pos = np.array([5, 5, 10])
viewer.camera().set_position(camera_pos)


light = pychrono.Lights()
light.set_position(np.array([10, 10, 10]))
light.set_diffuse_color(pychrono.Color(1, 1, 1))
light.set_specular_color(pychrono.Color(1, 1, 1))
scene.add_light(light)


ambient_light = pychrono.Lights()
ambient_light.set_position(np.array([10, 10, 10]))
ambient_light.set_diffuse_color(pychrono.Color(0.5, 0.5, 0.5))
scene.add_light(ambient_light)


pychrono.Simulation()