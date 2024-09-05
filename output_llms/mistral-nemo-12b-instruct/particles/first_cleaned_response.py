import pychrono as pc
import numpy as np
import random


pc.init()


scene = pc.Scene()
sim = pc.Simulation(scene)


renderer = pc.IrrlichtRenderer()
renderer.setScene(scene)
renderer.setWindowSize(800, 600)


emitter = pc.ParticleEmitter()
emitter.setEmissionRate(100)
emitter.setMaxParticles(1000)
emitter.setPosition([0, 0, 0])
emitter.setVelocity([0, 0, 0])
emitter.setAcceleration([0, -9.81, 0])  
emitter.setShape(pc.Shape.SPHERE)
emitter.setRadius(0.1)
emitter.setColor([1, 0, 0, 1])
scene.add(emitter)


for i in range(1000):
    particle = pc.Particle()
    particle.setPosition([random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)])
    particle.setVelocity([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)])
    particle.setOrientation(pc.Quaternion.random())
    particle.setShape(pc.Shape.SPHERE)
    particle.setRadius(0.05)
    particle.setColor([random.random(), random.random(), random.random(), 1])
    scene.add(particle)


def gravity(particle, dt):
    for other in scene.getParticles():
        if particle != other:
            dx = other.getPosition()[0] - particle.getPosition()[0]
            dy = other.getPosition()[1] - particle.getPosition()[1]
            dz = other.getPosition()[2] - particle.getPosition()[2]
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            force = 6.67430e-11 * (other.getMass() * particle.getMass()) / (distance**2)
            ax = force * dx / (particle.getMass() * distance)
            ay = force * dy / (particle.getMass() * distance)
            az = force * dz / (particle.getMass() * distance)
            particle.addForce([ax, ay, az])


sim.setGravity([0, -9.81, 0])
sim.setTimeStep(0.01)
sim.setMaxSteps(10000)


while not renderer.isWindowClosed():
    sim.step()
    for particle in scene.getParticles():
        gravity(particle, sim.getTimeStep())
    renderer.render()
    pc.sleep(0.01)


renderer.close()
pc.quit()