import chrono as c
from chrono import Irlicht


c.init()


sim = c.ChronoSimulation()


visualizer = Irlicht(sim, width=800, height=600)


class ParticleEmitter:
    def __init__(self, name, position, radius, velocity, orientation):
        self.name = name
        self.position = position
        self.radius = radius
        self.velocity = velocity
        self.orientation = orientation

    def emit_particles(self, num_particles):
        for _ in range(num_particles):
            
            particle = c.ChronoBodyDynamicsBox(name=f"{self.name}_particle",
                                                position=self.position,
                                                dimensions=(self.radius, self.radius, self.radius),
                                                mass=1.0,
                                                color=(1, 0, 0),
                                                opacity=0.5,
                                                friction=0.9)

            
            particle.set_velocity(self.velocity)
            particle.set_orientation(self.orientation)

            
            sim.Add(particle)


emitter = ParticleEmitter("Emitter", (0, 0, 0), 0.1, (0, 0, 0), (0, 0, 0))


def custom_gravitational_force(particle1, particle2):
    
    distance = c.Vec3(particle1.get_position()).subtract(particle2.get_position()).length()

    
    force = c.Vec3(0, 0, 0)
    if distance > 0:
        force = c.Vec3(0, 0, 0).add(c.Vec3(0, 0, -1).scale(1 / distance ** 2))

    
    particle1.apply_force(force, particle2.get_position())
    particle2.apply_force(-force, particle1.get_position())


sim.add_force(custom_gravitational_force)


num_particles = 100
for i in range(num_particles):
    position = c.Vec3(0, 0, 0).add(c.Vec3(0, 0, i * 0.1))
    velocity = c.Vec3(0, 0, 0).add(c.Vec3(0, 0, 0.1))
    orientation = c.Vec3(0, 0, 0).add(c.Vec3(0, 0, 0))
    emitter.emit_particles(1, position, 0.01, velocity, orientation)


sim.set_timestep(0.01)
sim.set_max_steps(1000)
sim.run()


visualizer.run()