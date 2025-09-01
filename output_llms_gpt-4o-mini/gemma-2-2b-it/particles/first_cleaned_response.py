import pychrono as chrono
import pychrono.irrlicht as irr
import random


chrono.init()


engine = irr.IrrlichtEngine()


world = chrono.World()


emitter = chrono.ParticleEmitter(world)


particle_count = 1000
particle_size = 0.1
particle_color = (1, 0, 0)  


for i in range(particle_count):
    
    shape = random.choice(["sphere", "cube"])

    
    position = chrono.Vector3(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))

    
    velocity = chrono.Vector3(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))

    
    orientation = chrono.Quaternion(random.uniform(0, 2 * chrono.PI), random.uniform(0, 2 * chrono.PI), random.uniform(0, 2 * chrono.PI))

    
    particle = chrono.Particle(world, shape, position, velocity, orientation, particle_size, particle_color)

    
    emitter.addParticle(particle)


gravitational_constant = 6.67430e-11  
mass_of_particle = 1.0e-27  


def gravitational_force(particle1, particle2):
    distance = chrono.Vector3.distance(particle1.position, particle2.position)
    force = gravitational_constant * mass_of_particle * (particle1.mass / distance ** 2) * (particle2.mass / distance ** 2)
    return force


for particle1 in emitter.particles:
    for particle2 in emitter.particles:
        if particle1 != particle2:
            force = gravitational_force(particle1, particle2)
            particle1.applyForce(force)
            particle2.applyForce(force)


engine.run()


engine.render()


engine.shutdown()