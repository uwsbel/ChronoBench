import pychrono as pychrono
import pychrono.viewer.iran as iran
import numpy as np

# Initialize PyChrono
pychrono.init()

# Create a scene
scene = pychrono.Scene()
scene.set_ground_plane(True)
scene.set_ground_properties(
    height=0,
    static=True,
    friction=1.0,
    bounce=0.3,
    mu=0.3
)

# Create the rover model
rover_body = pychrono.shape.Box(
    size=[1, 1.5, 1.5],
    name="rover",
    mass=2000,
    color=[1, 1, 0]
)
rover_wheels = []
wheel_radius = 0.3

# Create wheel assembly
wheel_assembly = pychrono.shape.Box(
    size=[1, 1, 0.3],
    name="wheel_assembly",
    mass=50,
    color=[0.5, 0.5, 0]
)

# Add wheels
for i in range(4):
    # Create wheel
    wheel = pychrono.shape.Cylinder(
        radius=wheel_radius,
        height=0.1,
        name=f"wheel_{i}",
        mass=10,
        color=[0.5, 0.5, 0]
    )
    
    # Create joint for wheel
    joint = pychrono.body.Joint(
        name=f"wheel_{i}_joint",
        type=pychrono.body.JointType.Sphere,
        parent=rover_body,
        child=wheel,
        axis=[0, 0, 1],
        limits=[-1.0, 1.0, -0.5, 0.5]
    )
    
    # Create motor for wheel
    motor = pychrono.control.Motor(
        name=f"wheel_{i}_motor",
        type=pychrono.control.MotorType.Revolute,
        joint=joint,
        limits=[-1.0, 1.0]
    )
    
    # Add wheel to assembly
    wheel_assembly.add_child(joint)
    wheel_assembly.add_child(motor)
    rover_body.add_child(wheel_assembly)
    rover_body.add_child(wheel)
    rover_wheels.append(wheel)
    wheel_assembly.set_position([0, 0, -wheel_radius])

# Add the rover to the scene
scene.add_body(rover_body)

# Set up camera
camera = iran.Camera()
camera.set_position([5, 5, 5])
camera.set.lookat([0, 0, 0])
scene.add_camera(camera)

# Set up lighting
light = iran.Light()
light.set_type(iran.LightType.Directional)
light.set_position([10, 10, 10])
light.set_color([1, 1, 1])
scene.add_light(light)

# Set shadow properties
scene.set_shadow_properties(
    shadow_map_size=1024,
    shadow_min_distance=0.1,
    shadow_max_distance=5.0
)

# Add visualization textures
ground_texture = pychrono.visualization.Texture(
    filename="ground_texture.png",
    width=512,
    height=512,
    format=pychrono.visualization.TextureFormat.Texture2D
)
rover_texture = pychrono.visualization.Texture(
    filename="rover_texture.png",
    width=512,
    height=512,
    format=pychrono.visualization.TextureFormat.Texture2D
)

# Set ground and rover textures
scene.get_ground().set_texture(ground_texture)
rover_body.set_texture(rover_texture)

# Set up motor control driver
motor_control = pychrono.control.MotorControlDriver(
    name="rover_motor",
    type=pychrono.control.MotorControlType.ABS,
    motor_list=[m for m in rover_body.get_children() if isinstance(m, pychrono.control.Motor)]
)

# Set up keyboard input
keyboard = pychrono.input.Keyboard()
keyboard.set_callback(motor_control)

# Set default parameters
motor_control.set_default_parameters(
    steering_gain=0.1,
    max_steering_angle=1.0,
    max_throttle=1.0
)

# Start simulation
simulation = pychrono.Simulation()
simulation.set_time_step(1.0 / 60.0)
simulation.set_callbacks(
    pre_step=lambda: motor_control.update(),
    post_step=lambda: None
)

# Run simulation
simulation.run()