import PyChrono as pc
import Irrlicht as irl
import numpy as np


simulation = pc.Simulation()
simulation.init()


particle_system = pc.ParticleSystem()
particle_system.position = np.array([0, 0, 0])
particle_system.velocity = np.zeros(3)
particle_system.force = np.zeros(3)
simulation.add_system(particle_system)


particle_emitter = pc.ParticleEmitter()
particle_emitter.position = np.array([0, 0, 0])
particle_emitter.color = np.random.rand(3) * 255  
particle_emitter.shape = pc.Sphere(0.5, 0.5, 0.5)  
particle_emitter.emission_radius = 5.0  
particle_emitter.emit = True
simulation.add_emitter(particle_emitter)


scene = irl.Scene()
camera = scene.createCamera("camera", 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 45, 0, 0, 0)
light = scene.createLight("light", 0, 0, 0, 0, 0, 0, 100, 45, 0, 0, 0)
particle_system.add_component(pc.VisualizationComponent(scene, camera, 1000))


particle_system.position_component.position_enabled = True
particle_system.velocity_component.velocity_enabled = True
particle_system.force_component.force_enabled = True


def gravitational_force(particle):
    pos1 = particle.position
    pos2 = particle_system.position
    r = np.linalg.norm(pos1 - pos2)
    if r > 1e-8:
        force = 50.0 * (pos2 - pos1) / r**2
        particle.force = force

particle_system.add_component(pc.CustomForceComponent(gravitational_force))


def key_pressed(key):
    if key == 32:  
        particle_system.position = np.array([0, 0, 0])
        particle_system.velocity = np.zeros(3)
        particle_system.force = np.zeros(3)

simulation.key_pressed.AddListener(key_pressed)


while True:
    simulation.step()
    scene.render()