# Import necessary PyChrono and Irrlicht modules
from pychrono.core import *
from pychrono.irrlicht_utils import *

# Initialize the PyChrono environment
chrono_init()

# Create the physical world
world = ChronoData.GetDefaultPhysicsWorld()

# Create a ground body
ground = create_ground_body(world)

# Create the pendulum body
pendulum_mass = 1.0  # Mass of the pendulum bob in kg
pendulum_inertia = create_inertia(world, pendulum_mass, 0.1, 0.1, 0.1)
pendulum_shape = create_sphere_shape(world, 0.1)
pendulum_body = create_dynamic_body(world, pendulum_shape, pendulum_inertia)

# Set the initial position and orientation of the pendulum
pendulum_body.set_position(Vector3d(0.0, 0.0, 0.5))  # hanging from the ground
pendulum_body.set_orientation(chrono_euler_to_quaternion(0, 0, 0, M_PI / 2))

# Create a revolute joint to connect the pendulum to the ground
revolute_joint = create_revolute_joint(world, pendulum_body, ground, Vector3d(0, 0, 0), Vector3d(0, 0, 0), M_PI / 2)

# Set up the visualization system
viewer = create_default_irrlicht_viewer(640, 480)

# Set up the rendering system
chrono_video_renderer_add_to_viewer(viewer)
chrono_video_renderer_set_background_color(viewer, 0.5, 0.5, 0.5)

# Set up the simulation loop
while irrlicht_application_run(viewer) and irrlicht_application_is_running():
    world.Step(chrono_inv_real_time_step(1.0 / 60.0, true)  # Simulate at 60Hz

    # Render the scene
    for entity in world.GetEntities():
        if is_a(entity, b2_dynamic_body):
            # Render the dynamic bodies
            entity.Draw(viewer.get_render_device())

    # Log the pendulum's position and velocity
    pendulum = world.GetBody(pendulum_body)
    position = pendulum.get_position()
    velocity = pendulum.get_linear_velocity()
    print(f"Pendulum position: {position}, Pendulum velocity: {velocity}")

    # Update the simulation
    world.Step(chrono_inv_real_time_step(1.0 / 60.0, true)

# Clean up
chrono_shutdown()