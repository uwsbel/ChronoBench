import pychrono as pc
import numpy as np
import Irrlicht as ir


pc.init()
ir.init()


sim = pc.Simulation(pc.SolverType.PC_Solver, 0.1)  
sim.set_solver_tolerance(1e-8)


particle_mass = 1.0
particle_size = 0.1
gravity_constant = 100.0  


def create_particle():
    
    pos = np.array([np.random.uniform(-10, 10) for _ in range(3)])
    
    
    vel = np.array([np.random.uniform(-10, 10) for _ in range(3)])
    vel = vel / np.linalg.norm(vel) * 10  
    
    
    orient = np.random.uniform(0, 2*np.pi, 3)
    orient = np.array([orient[0], orient[1], orient[2]])
    
    
    particle = pc.Particle()
    particle.set_mass(particle_mass)
    particle.set_shape(pc.Sphere(particle_size))
    particle.set_position(pos)
    particle.set_velocity(vel)
    particle.set_orientation(orient)
    return particle


renderer = ir.Renderer()
camera = ir.Camera()
renderer.set_camera(camera)
renderer.set_background_color(ir.Color(0, 0, 0))


particle_shader = ir.Shader()
particle_shader.set_vertex_shader(
    
)


force_shader = ir.Shader()
force_shader.set_vertex_shader(
    
)

renderer.set_shader_for_particles(particle_shader)
renderer.set_shader_for_lines(force_shader)


while True:
    
    sim.clear_objects()
    
    
    particle = create_particle()
    sim.add_object(particle)
    
    
    particles = sim.get_objects()
    num_particles = len(particles)
    
    for i in range(num_particles):
        for j in range(i + 1, num_particles):
            p1 = particles[i]
            p2 = particles[j]
            
            
            r = np.linalg.norm(p1.get_position() - p2.get_position())
            
            if r > 1e-8:  
                force = gravity_constant * particle_mass * p1.get_mass() / (r ** 2)
                acc1 = force / (p1.get_mass()) * (p2.get_position() - p1.get_position()) / (r ** 2)
                acc2 = force / (p2.get_mass()) * (p1.get_position() - p2.get_position()) / (r ** 2)
                
                p1.add_force(acc1)
                p2.add_force(acc2)
    
    
    renderer.begin_frame()
    renderer.render()
    
    
    keys = ir.Keyboard()
    if keys.key_pressed(ir.Key('e')):
        particle = create_particle()
        sim.add_object(particle)
    
    renderer.end_frame()
    
    
    if len(particles) > 1000:
        sim.remove_objects(sim.get_objects()[-1000:])