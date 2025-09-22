import pychrono as chrono
import pychrono.irrlicht as irrlicht
import numpy as np
import random


chrono.init()


engine = irrlicht.IrrlichtEngine()
engine.createWindow("Gravitational Attraction Simulation", 800, 600)


emitter = chrono.ParticleEmitter()
emitter.setParticleCount(100)
emitter.setParticleSize(0.1)


particle_properties = chrono.ParticleProperties()
particle_properties.setShape(chrono.Shape.Sphere)
particle_properties.setMass(1.0)
particle_properties.setColor(chrono.Color.Red)


G = 6.67430e-11  


positions = np.random.rand(100, 3) * 100
velocities = np.random.rand(100, 3) * 10
orientations = np.random.rand(100, 3) * 2 * np.pi


for i in range(100):
    emitter.addParticle(
        chrono.Particle(
            positions[i],
            velocities[i],
            orientations[i],
            particle_properties
        )
    )


scene = engine.getScene()
scene.setCameraPosition(chrono.Vector3(0, 0, 10))


engine.render()


while True:
    
    for particle in emitter.getParticles():
        particle.update(G, engine.getDeltaTime())

    
    engine.update()

    
    engine.render()