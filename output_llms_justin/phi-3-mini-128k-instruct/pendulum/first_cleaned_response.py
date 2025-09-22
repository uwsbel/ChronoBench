from pychrono.core import *
from pychrono.irrlicht_utils import *


chrono_init()


world = ChronoData.GetDefaultPhysicsWorld()


ground = create_ground_body(world)


pendulum_mass = 1.0  
pendulum_inertia = create_inertia(world, pendulum_mass, 0.1, 0.1, 0.1)
pendulum_shape = create_sphere_shape(world, 0.1)
pendulum_body = create_dynamic_body(world, pendulum_shape, pendulum_inertia)


pendulum_body.set_position(Vector3d(0.0, 0.0, 0.5))  
pendulum_body.set_orientation(chrono_euler_to_quaternion(0, 0, 0, M_PI / 2))


revolute_joint = create_revolute_joint(world, pendulum_body, ground, Vector3d(0, 0, 0), Vector3d(0, 0, 0), M_PI / 2)


viewer = create_default_irrlicht_viewer(640, 480)


chrono_video_renderer_add_to_viewer(viewer)
chrono_video_renderer_set_background_color(viewer, 0.5, 0.5, 0.5)


while irrlicht_application_run(viewer) and irrlicht_application_is_running():
    world.Step(chrono_inv_real_time_step(1.0 / 60.0, true)  

    
    for entity in world.GetEntities():
        if is_a(entity, b2_dynamic_body):
            
            entity.Draw(viewer.get_render_device())

    
    pendulum = world.GetBody(pendulum_body)
    position = pendulum.get_position()
    velocity = pendulum.get_linear_velocity()
    print(f"Pendulum position: {position}, Pendulum velocity: {velocity}")

    
    world.Step(chrono_inv_real_time_step(1.0 / 60.0, true)


chrono_shutdown()