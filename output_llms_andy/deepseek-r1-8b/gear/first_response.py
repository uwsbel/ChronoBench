import pychrono as pyc
import numpy as np

# Initialize PyChrono environment
pyc.init()
scene = pyc.Scene()
camera = pyc.Camera()
renderer = pyc.IrrlichtRenderer()
renderer.set_shadow_map_enabled(True)
renderer.set_light_color(1, 1, 1)  # Enable lighting

# Add a point light
light = pyc.Light()
light.set_position(pyc.Vector3(5, 5, 5))
light.set_color(pyc.Vector3(1, 1, 1))
light.set_radius(5)
scene.add_light(light)

# Add a directional light
dir_light = pyc.DirectionalLight()
dir_light.set_position(pyc.Vector3(5, 5, 5))
dir_light.set_color(pyc.Vector3(1, 1, 1))
dir_light.set_radius(5)
scene.add_light(dir_light)

# Add a ground plane
ground = pyc.RigidBody()
ground.set_geometry(pyc.BoxGeometry(50, 1, 1))
ground.set_position(pyc.Vector3(0, 0, 0))
scene.add_rigidbody(ground)

# Add camera
camera.set_position(pyc.Vector3(0, 0, 5))
camera.look_at(pyc.Vector3(0, 0, 0))
renderer.set_camera(camera)

# Add a fixed truss
truss = pyc.RigidBody()
truss.set_geometry(pyc.BoxGeometry(2, 2, 2))
truss.set_position(pyc.Vector3(0, 0, 0))
scene.add_rigidbody(truss)

# Add a rotating bar
rotating_bar = pyc.RigidBody()
rotating_bar.set_geometry(pyc.CylinderGeometry(0.5, 0.5, 2, 32))
rotating_bar.set_position(pyc.Vector3(0, 0, 2))
rotating_bar.set_mass(0)
# Attach rotating bar to truss with revolute joint around X-axis
joint = pyc.RevolteJoint()
joint.set_parent(truss)
joint.set_child(rotating_bar)
joint.set_rotation_x(pyc.Vector3(np.pi/2))
joint.set_axis(pyc.Vector3(1, 0, 0))
joint.set_damping(0.1)
scene.add_joint(joint)

# Add two epicyclic gears
gear1 = pyc.RigidBody()
gear1.set_geometry(pyc.CylinderGeometry(0.5, 0.5, 2, 32))
gear1.set_position(pyc.Vector3(0, 1, 2))
gear1.set_mass(0)
gear2 = pyc.RigidBody()
gear2.set_geometry(pyc.CylinderGeometry(0.5, 0.5, 2, 32))
gear2.set_position(pyc.Vector3(0, -1, 2))
gear2.set_mass(0)

# Add gear motor
motor = pyc.RigidBody()
motor.set_geometry(pyc.BoxGeometry(1, 1, 1))
motor.set_position(pyc.Vector3(0, 0, 2))
motor.set_mass(0)
motor.set_rotation_speed(1)  # Initial rotation speed

# Connect gears to the motor
joint1 = pyc.RevolteJoint()
joint1.set_parent(motor)
joint1.set_child(gear1)
joint1.set_rotation_x(pyc.Vector3(np.pi/2))
joint1.set_axis(pyc.Vector3(1, 0, 0))
joint1.set_damping(0.1)
scene.add_joint(joint1)

joint2 = pyc.RevolteJoint()
joint2.set_parent(motor)
joint2.set_child(gear2)
joint2.set_rotation_x(pyc.Vector3(np.pi/2))
joint2.set_axis(pyc.Vector3(1, 0, 0))
joint2.set_damping(0.1)
scene.add_joint(joint2)

# Connect gears together
joint3 = pyc.HelicalJoint()
joint3.set_parent(gear1)
joint3.set_child(gear2)
joint3.set_rotation_x(pyc.Vector3(0, 1, 0))
joint3.set_translation_x(pyc.Vector3(0, 0, 0))
joint3.set_damping(0.1)
scene.add_joint(joint3)

# Add materials
truss_mat = pyc.Material()
truss_mat.set_diffuse_color(pyc.Vector3(0.5, 0.5, 0.5))
truss.set_material(truss_mat)

gear_mat = pyc.Material()
gear_mat.set_diffuse_color(pyc.Vector3(0, 1, 0))
gear1.set_material(gear_mat)
gear2.set_material(gear_mat)

motor_mat = pyc.Material()
motor_mat.set_diffuse_color(pyc.Vector3(1, 0, 0))
motor.set_material(motor_mat)

# Set visualization settings
renderer.set_light_color(1, 1, 1)
renderer.set_diffuse_lighting(True)
renderer.set_shadows(True)

# Add simulation loop
time_step = 0.001
simulation_loop = pyc.SimulationLoop()
simulation_loop.set_time_step(time_step)

def simulate():
    simulation_loop.start()
    while True:
        simulation_loop.step()
        scene.update()

# Add key press hook to reset simulation
def key_press_hook(key):
    if key == 'r':
        simulation_loop.reset()
        scene.clear()
        # Reset all elements
        # (Add reset logic for specific elements if needed)
        pass

simulation_loop.set_key_press_hook(key_press_hook)

# Start the simulation
simulate()