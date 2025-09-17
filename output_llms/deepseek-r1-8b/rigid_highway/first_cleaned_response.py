import pychrono as pychrono
import pychrono.vehicles as vehicles
import pychrono.render as render
from pychrono.input import Input


sim = pychrono.Simulation()
render = pychrono.render.Render()
sim.set_render(render)


ground = pychrono.objects.Plane()
ground.set_name("Ground")
ground.set_position([0, 0, 0])
ground.set_size([100, 100, 0.1])
ground.set_material(pychrono.materials.Material(1, 1, 1, 0.5))
sim.add_object(ground)


hmmwv = vehicles.vehicles.Tank()
hmmwv.set_name("HMMWV")
hmmwv.set_position([0, 0, 0.5])
hmmwv.set_orientation(pychrono.Orientation(0, 1, 0))
hmmwv.set_contact_method("wheels")
hmmwv.set_tire_model("TMEASY")
hmmwv.set_mass(1000)
hmmwv.set_friction(0.5)



cabin = pychrono.objects.Box()
cabin.set_name("Cabin")
cabin.set_position([hmmwv.get_position()[0], hmmwv.get_position()[1], 1.5])
cabin.set_size([2, 1.5, 1.5])
cabin.set_material(pychrono.materials.Material(1, 1, 1, 0.5))
hmmwv.add_object(cabin)


gun = pychrono.objects.Box()
gun.set_name("Gun")
gun.set_position([hmmwv.get_position()[0], 0, 2])
gun.set_size([1, 1, 1])
gun.set_material(pychrono.materials.Material(1, 0, 0, 0.5))
hmmwv.add_object(gun)


wheel_positions = [
    [hmmwv.get_position()[0] - 1.5, hmmwv.get_position()[1], 0.5],
    [hmmwv.get_position()[0] + 1.5, hmmwv.get_position()[1], 0.5],
    [hmmwv.get_position()[0] + 1.5, hmmwv.get_position()[1], 0.5],
    [hmmwv.get_position()[0] - 1.5, hmmwv.get_position()[1], 0.5]
]

for pos in wheel_positions:
    wheel = pychrono.objects.Sphere()
    wheel.set_name(f"Wheel_{i}")
    wheel.set_position(pos)
    wheel.set_radius(0.5)
    wheel.set_mass(5)
    wheel.set_friction(1)
    hmmwv.add_object(wheel)


terrain = pychrono.objects.Terrain()
terrain.set_name("Terrain")
terrain.add_mesh("Highway_col.obj", "Highway_vis.obj", 0, 0, 0, 0)
terrain.set_ground_plane(sim.get_object("Ground"))
sim.add_object(terrain)


input_system = Input()
input_system.set_steering_sensitivity(0.1)
input_system.set_throttle_sensitivity(0.1)
input_system.set_brake_sensitivity(0.1)
sim.set_input(input_system)


hmmwv.set_max_speed(20)
hmmwv.set_max_acceleration(10)
hmmwv.set_roll_stiffness(100)
hmmwv.set_pitch_stiffness(100)


render.set_camera_position([10, 10, 10])
render.set_field_of_view(45)
render.set_up_vector([0, 1, 0])
render.set_view_matrix()
render.set_projection_matrix()


hmmwv.add_mesh("Cabin", "cabin visualization.gtx", 1, 1, 1, 1)
hmmwv.add_mesh("Gun", "gun visualization.gtx", 1, 1, 1, 1)
for wheel in hmmwv.get_objects_of_type("Wheel"):
    wheel.add_mesh("Wheel", "wheel visualization.gtx", 0.5, 0.5, 0.5, 1)


sim.start()
while True:
    
    speed = 0
    if input_system.get_key('accelerator'):
        speed += 5
    if input_system.get_key('brake'):
        speed -= 5
    if input_system.get_key('left'):
        hmmwv.apply_force("left", 100)
    if input_system.get_key('right'):
        hmmwv.apply_force("right", 100)
    
    
    sim.update()
    
    
    render.render_to_window(50)