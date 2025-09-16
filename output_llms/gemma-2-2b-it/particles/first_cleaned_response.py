import pychrono as chrono
import pychrono.core as core
import pychrono.irrlicht as irrlicht
import random
import numpy as np


chrono.Init()


world = chrono.World()


emitter = chrono.ParticleEmitter(world)
emitter.SetGravity(chrono.Vector3(0, 0, -9.81))  
emitter.SetParticleCount(100)  


particle_radius = 0.1
particle_mass = 1.0


for i in range(emitter.GetParticleCount()):
    position = chrono.Vector3(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))
    velocity = chrono.Vector3(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    orientation = chrono.Quaternion(random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi))
    emitter.AddParticle(position, velocity, orientation)


irrlicht_scene = irrlicht.IrrlichtScene()


for particle in emitter.GetParticles():
    irrlicht_scene.AddParticle(particle)


irrlicht_scene.SetCameraPosition(chrono.Vector3(0, 0, 10))
irrlicht_scene.SetCameraRotation(chrono.Quaternion(0, 0, 0, 1))


irrlicht_system = irrlicht_scene.CreateIrrlichtSystem()


chrono.Run()