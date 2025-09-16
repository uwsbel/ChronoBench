import pychrono as chrono
import numpy as np


chrono.setChronoDataPath("path/to/chrono_data")


sim = chrono.ChSimulation()
scene = chrono.ChIrrApp(sim, 'Particle Attraction Simulation')
scene.addLight(chrono.ChLightDir())
scene.setCamera(chrono.ChVectorD(1, 1, 1), chrono.ChVectorD(0, 0, 0))


emitter = chrono.ChParticleEmitter()
emitter.SetEmissionRate(100)  
emitter.SetEmissionDuration(chrono.GetChronoTimeStep())  
emitter.SetEmissionDirection(chrono.ChVectorD(0, 1, 0))  
emitter.SetEmissionPosition(chrono.ChVectorD(0, 1, 0))  
sim.Add(emitter)


particle_system = chrono.ChParticleSystem()
particle_system.SetParticleRadius(0.05)  
particle_system.SetMaxNumParticles(1000)  
particle_system.SetGravity(chrono.ChVectorD(0, -9.81, 0))  
sim.Add(particle_system)


def custom_gravity(particle, force):
    position = particle.GetPos()
    force[0] = 0
    force[1] = -9.81 * position.y
    force[2] = 0


particle_system.SetCustomGravityFunction(custom_gravity)


while scene.Run():
    sim.DoStep()
    scene.Render()