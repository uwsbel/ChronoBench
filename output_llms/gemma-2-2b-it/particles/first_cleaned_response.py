import pychrono as pchrono
import numpy as np
from irrlicht import irrlicht
from irrlicht import irrlichtEngine


pchrono.init()


engine = irrlichtEngine()


particle_emitter = pchrono.ParticleEmitter()


particle_properties = pchrono.ParticleProperties()
particle_properties.shape = pchrono.Shape.sphere
particle_properties.size = 0.1
particle_properties.color = (1, 0, 0)  


particle = pchrono.Particle()
particle.properties = particle_properties


particle.position = np.array([0, 0, 0])
particle.velocity = np.array([0, 0, 0])
particle.orientation = np.array([0, 0, 0])


particle_emitter.add_particle(particle)


time_step = 0.01


gravity = pchrono.Force(pchrono.Vector3(0, -9.81, 0))


scene = pchrono.Scene()


scene.add_component(particle_emitter)


scene.add_force(gravity)


engine.run()