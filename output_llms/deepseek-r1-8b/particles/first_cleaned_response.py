import PyChrono as pc
import numpy as np
import Irrlicht


simulation = pc.Simulation()
pc.set_physics_settings(simulation, gravity=0.001)  


class ParticleEmitter:
    def __init__(self, simulation):
        self.simulation = simulation
        self.particles = []
        self.shape_options = ['circle', 'square', 'random_shape']
        self.position_options = np.random.uniform(-10, 10, size=(100,))
        self.velocity_options = np.random.uniform(-10, 10, size=(100,))
        self.orientation_options = np.random.uniform(0, 2*np.pi, size=(100,))
    
    def emit(self):
        for _ in range(100):
            shape = np.random.choice(self.shape_options)
            pos = np.random.choice(self.position_options)
            vel = np.random.choice(self.velocity_options)
            orient = np.random.choice(self.orientation_options)
            
            
            particle = pc.Particle()
            particle.position = pos
            particle.velocity = vel
            particle.orientation = orient
            particle.shape = shape
            self.simulation.add_particle(particle)
            self.particles.append(particle)


renderer = Irrlicht.Renderer(simulation, camera=pc.Camera())
renderer.scene.clear()
light = Irrlicht.Light()
light.position = (10, 10, 10)
renderer.add_light(light)
renderer.camera.position = (50, 50, 50)


def gravity_force(particle1, particle2):
    r = np.linalg.norm(particle1.position - particle2.position)
    if r < 1e-10:
        return 0
    m1 = particle1.mass
    m2 = particle2.mass
    G = 0.001  
    F = G * m1 * m2 / (r**2)
    F_vector = (particle2.position - particle1.position) * F / (r**2) * particle1.mass
    return F_vector

simulation.add_particle_system(
    particles=[particle for particle in simulation.particles],
    force_func=gravity_force,
    shape=particle.shape for particle in simulation.particles,
    mass=1.0 for particle in simulation.particles
)


simulation.run(1000, 1000000)


def key_event_handler(key):
    if key.key_code == ord('W'):
        renderer.camera.position[1] += 5
    if key.key_code == ord('S'):
        renderer.camera.position[1] -= 5
    if key.key_code == ord('A'):
        renderer.camera.position[0] -= 5
    if key.key_code == ord('D'):
        renderer.camera.position[0] += 5
    if key.key_code == ord('R'):
        renderer.camera.look_at = np.array([0, 0, 0])

simulation.event_handler = key_event_handler


while True:
    simulation.step()
    renderer.render()
    renderer.scene.clear()
    for particle in simulation.particles:
        renderer.draw_sphere(particle.position, particle.radius=0.1, color=(1,1,1))