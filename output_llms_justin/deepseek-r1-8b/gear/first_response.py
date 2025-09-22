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

# Create fixed truss
truss = pyc.RigidBody()
truss.set_geometry(pyc.Box3d(1, 1, 1))
truss.set_position(pyc.Vector3(0, 0, 0))
truss.set_color(pyc.Color(0.5, 0.5, 0.5))
scene.add_body(truss)

# Create rotating bar
rotating_bar = pyc.RigidBody()
rotating_bar.set_geometry(pyc.Cylinder(0.1, 0.1, 1))
rotating_bar.set_position(pyc.Vector3(0, 0, 2))
rotating_bar.set_color(pyc.Color(0, 1, 0))
# Attach to truss with revolute joint
joint = pyc.RevolteJoint()
joint.set_parent(truss)
joint.set_child(rotating_bar)
joint.set_rotation(pyc.Vector3(0, 0, 1))  # Initial rotation
joint.set_axis(pyc.Vector3(1, 0, 0))  # Rotation axis (X-axis)
joint.set_properties(pyc.RevolteJoint.ROTX, 0.0, 0.0, 0.0, 0.0, 1.0)  # Allow rotation around X-axis

# Create epicyclic gears
# Gear 1
gear1 = pyc.RigidBody()
gear1.set_geometry(pyc.Cylinder(0.1, 0.1, 1))
gear1.set_position(pyc.Vector3(0, 0, 1))
gear1.set_color(pyc.Color(0, 1, 0))
# Attach to rotating bar
joint_gear1 = pyc.RevolteJoint()
joint_gear1.set_parent(rotating_bar)
joint_gear1.set_child(gear1)
joint_gear1.set_rotation(pyc.Vector3(0, 0, 1))  # Initial rotation
joint_gear1.set_axis(pyc.Vector3(1, 0, 0))  # Rotation axis (X-axis)
joint_gear1.set_properties(pyc.RevolteJoint.ROTX, 0.0, 0.0, 0.0, 0.0, 1.0)  # Allow rotation around X-axis

# Gear 2
gear2 = pyc.RigidBody()
gear2.set_geometry(pyc.Cylinder(0.1, 0.1, 1))
gear2.set_position(pyc.Vector3(0, 0, -1))
gear2.set_color(pyc.Color(0, 1, 0))
# Attach to rotating bar
joint_gear2 = pyc.RevolteJoint()
joint_gear2.set_parent(rotating_bar)
joint_gear2.set_child(gear2)
joint_gear2.set_rotation(pyc.Vector3(0, 0, 1))  # Initial rotation
joint_gear2.set_axis(pyc.Vector3(1, 0, 0))  # Rotation axis (X-axis)
joint_gear2.set_properties(pyc.RevolteJoint.ROTX, 0.0, 0.0, 0.0, 0.0, 1.0)  # Allow rotation around X-axis

# Create motor
motor = pyc.RigidBody()
motor.set_geometry(pyc.Box3d(0.5, 0.5, 0.5))
motor.set_position(pyc.Vector3(0, 0, 0.5))
motor.set_color(pyc.Color(1, 0, 0))
# Attach to gear1
joint_motor = pyc.RevolteJoint()
joint_motor.set_parent(gear1)
joint_motor.set_child(motor)
joint_motor.set_rotation(pyc.Vector3(0, 0, 1))  # Initial rotation
joint_motor.set_axis(pyc.Vector3(1, 0, 0))  # Rotation axis (X-axis)
joint_motor.set_properties(pyc.RevolteJoint.ROTX, 0.0, 0.0, 0.0, 0.0, 1.0)  # Allow rotation around X-axis
motor.set_rotational_speed(1.0)  # Set constant rotation speed

# Create ground plane
ground = pyc.Plane3d()
ground.set_position(pyc.Vector3(0, 0, -1))
ground.set_rotation(pyc.Vector3(0, 1, 0))
scene.add_body(ground)

# Set camera position
camera.set_position(pyc.Vector3(0, 0, 3))
camera.set.look_at(pyc.Vector3(0, 0, 0))

# Add materials
material_truss = pyc.Material()
material_truss.set_diffuse_color(pyc.Color(0.5, 0.5, 0.5))
truss.set_material(material_truss)

material_gear = pyc.Material()
material_gear.set_diffuse_color(pyc.Color(0, 1, 0))
gear1.set_material(material_gear)
gear2.set_material(material_gear)

material_motor = pyc.Material()
material_motor.set_diffuse_color(pyc.Color(1, 0, 0))
motor.set_material(material_motor)

# Add visualization
scene.add_body(truss)
scene.add_body(rotating_bar)
scene.add_body(gear1)
scene.add_body(gear2)
scene.add_body(motor)

# Set simulation time step
t = 0.001

# Visualization settings
renderer.set_shadow_map_enabled(True)
renderer.set_light_color(1, 1, 1)
renderer.set_light_radius(5)

# Add ground plane
ground_plane = pyc.Plane3d()
ground_plane.set_position(pyc.Vector3(0, 0, -1))
ground_plane.set_rotation(pyc.Vector3(0, 1, 0))
scene.add_body(ground_plane)

# Add camera
scene.add_camera(camera)

# Set key press hook for reset simulation
def key_pressed_callback(key):
    if key == 'r':
        # Reset simulation
        pyc.reset()
        # Reset motor rotation speed
        motor.set_rotational_speed(1.0)

pyc.add_key_press_hook(key_pressed_callback)

# Set simulation loop
while True:
    pyc.step(t)
    renderer.render(scene, camera, 256, 256)
    # Optional: Add rendering settings
    # renderer.set_lighting_enabled(True)
    # renderer.set_shadows_enabled(True)
    # renderer.set_depth_of_field(1000)
    # renderer.set_fog_enabled(True)
    
    # Add debugging print statements if needed
    print(f"Simulation Time: {pyc.get_time()}")
    
    # Optional: Add keyboard controls
    if pyc.is_key_pressed('r'):
        pyc.reset()
        motor.set_rotational_speed(1.0)
    
    # Add any other desired functionality here