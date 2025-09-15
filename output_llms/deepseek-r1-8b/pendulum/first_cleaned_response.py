import pychrono as pychrono
import pychrono.trajectory as trajectory
import pychrono.visualization.irrlicht as irrlicht


pychrono.initialize()
simulation = pychrono.Simulation()
visualization = pychrono.Visualization()
renderer = visualization.createRenderer()


ground_body = pychrono.Body()
ground_body.set_mass(0)
ground_body.set_position(pychrono.Vec3d(0, 0, 0))
simulation.add_body(ground_body)


mass = 1.0
length = 5.0
inertia = 1/5  


pendulum_body = pychrono.Body()
pendulum_body.set_mass(mass)
pendulum_body.set_inertia(inertia)
pendulum_body.set_position(pychrono.Vec3d(0, 5, 0))  


joint = pychrono.RevoluteJoint()
joint.set_damping(0.1)
joint.set_rotation_limit(1.0)  
simulation.add_joint(joint)
simulation.add_constraint(joint, pendulum_body, ground_body)


visualization.set_renderer(renderer)
camera = pychrono.Camera()
camera.set_position(pychrono.Vec3d(0, 10, 30))
camera.set_lookat(pychrono.Vec3d(0, 0, 0))
renderer.set_camera(camera)
renderer.set_light(0, pychrono.Vec3d(1, 1, 1), pychrono.Vec3d(0, 0, 0))
renderer.set_light(1, pychrono.Vec3d(-1, -1, -1), pychrono.Vec3d(0, 0, 0))


simulation.add_force(pychrono.Vec3d(0, -9.81, 0), pendulum_body)
simulation.add_torque(pychrono.Vec3d(0, 0, 0), pendulum_body, 1.0)


log = pychrono.Logging()
log.set_path("pendulum_log")
log.add_scalar("position_x", True)
log.add_scalar("position_y", True)
log.add_scalar("position_z", True)
log.add_scalar("velocity_x", True)
log.add_scalar("velocity_y", True)
log.add_scalar("velocity_z", True)


while True:
    simulation.step()
    
    
    
    
    
    log.log()
    
    
    visualization.render()
    
    
    if renderer.keys_pressed().get('q'):
        break


pychrono.finalize()