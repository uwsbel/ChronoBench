import pychrono as pc
import numpy as np
import Irrlicht as ir

# Initialize PyChrono
pc.init()
ir.init()

# Simulation parameters
sim = pc.Simulation(pc.SolverType.PC_Solver, 0.1)  # Time step 0.1 ms
sim.set_solver_tolerance(1e-8)

# Particle properties
particle_mass = 1.0
particle_size = 0.1
gravity_constant = 100.0  # Adjust this to modify gravitational strength

# Particle emitter function
def create_particle():
    # Random position
    pos = np.array([np.random.uniform(-10, 10) for _ in range(3)])
    
    # Random velocity
    vel = np.array([np.random.uniform(-10, 10) for _ in range(3)])
    vel = vel / np.linalg.norm(vel) * 10  # Normalize velocity
    
    # Random orientation
    orient = np.random.uniform(0, 2*np.pi, 3)
    orient = np.array([orient[0], orient[1], orient[2]])
    
    # Create particle
    particle = pc.Particle()
    particle.set_mass(particle_mass)
    particle.set_shape(pc.Sphere(particle_size))
    particle.set_position(pos)
    particle.set_velocity(vel)
    particle.set_orientation(orient)
    return particle

# Visualization setup
renderer = ir.Renderer()
camera = ir.Camera()
renderer.set_camera(camera)
renderer.set_background_color(ir.Color(0, 0, 0))

# Create shader for particle visualization
particle_shader = ir.Shader()
particle_shader.set_vertex_shader(
    """
    void main() {
        vec3 pos = worldPos;
        vec3 vel = worldVel;
        vec3 force = worldForce;
        float speed = length(vel);
        float force_mag = length(force);
        vec3 color = vec3(
            (force_mag * 0.5) / (speed * 0.5 + 1.0),
            (force_mag * 0.5) / (speed * 0.5 + 1.0),
            (speed / (speed * 0.5 + 1.0))
        );
        color *= 0.5 + 0.5 * sin(time) * 0.5;
        gl_FragColor = vec4(color, 1.0);
    }
    """
)

# Visualization setup for forces
force_shader = ir.Shader()
force_shader.set_vertex_shader(
    """
    void main() {
        vec3 force = worldForce;
        vec3 color = vec3(
            (length(force) * 0.5) / (length(worldVel) * 0.5 + 1.0),
            (length(force) * 0.5) / (length(worldVel) * 0.5 + 1.0),
            0.0
        );
        gl_FragColor = vec4(color, 1.0);
    }
    """
)

renderer.set_shader_for_particles(particle_shader)
renderer.set_shader_for_lines(force_shader)

# Simulation loop
while True:
    # Clear previous objects
    sim.clear_objects()
    
    # Emit new particle
    particle = create_particle()
    sim.add_object(particle)
    
    # Apply gravity between particles
    particles = sim.get_objects()
    num_particles = len(particles)
    
    for i in range(num_particles):
        for j in range(i + 1, num_particles):
            p1 = particles[i]
            p2 = particles[j]
            
            # Calculate distance
            r = np.linalg.norm(p1.get_position() - p2.get_position())
            
            if r > 1e-8:  # Avoid division by zero
                force = gravity_constant * particle_mass * p1.get_mass() / (r ** 2)
                acc1 = force / (p1.get_mass()) * (p2.get_position() - p1.get_position()) / (r ** 2)
                acc2 = force / (p2.get_mass()) * (p1.get_position() - p2.get_position()) / (r ** 2)
                
                p1.add_force(acc1)
                p2.add_force(acc2)
    
    # Visualize simulation
    renderer.begin_frame()
    renderer.render()
    
    # Handle key input for emitting particles
    keys = ir.Keyboard()
    if keys.key_pressed(ir.Key('e')):
        particle = create_particle()
        sim.add_object(particle)
    
    renderer.end_frame()
    
    # Limit number of particles
    if len(particles) > 1000:
        sim.remove_objects(sim.get_objects()[-1000:])