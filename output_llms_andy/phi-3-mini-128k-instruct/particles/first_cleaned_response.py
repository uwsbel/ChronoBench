from pychrono.pychrono import *
from pychrono.irrlicht import *


chrono_arena = ChronoArena()
chrono_arena.SetGravity(chrono_arena.GetGravity())


application = Application(chrono_arena, True, 'ParticleSimulation')
application.SetWindowSize(800, 600)
application.SetGamma(0.25)
application.SetGammaBlack()


particle_emitter = ParticleEmitter()


particle_system = chrono_arena.GetParticleSystem()
particle_system.SetEmitter(particle_emitter)


def custom_gravitational_force(particle, acceleration):
    
    G = 6.67430e-11
    mass = 1.0  
    acceleration = G * mass / (particle.GetPosition().LengthSquared() * 1e-3)  
    return chrono_arena.GetGravitation


def custom_gravitational_force(particle, acceleration):
    G = 6.67430e-11  
    mass = 1.0  
    distance_squared = particle.GetPosition().LengthSquared()
    if distance_squared > 0:  
        acceleration = G * mass / distance_squared * particle.GetPosition().Unit()
    return acceleration



chrono_arena.SetCustomGravity(custom_gravitational_force)


application.SetSimulate(True)
application.SetStep(1e-3)
application.BeginScene()
application.AddLogicalDevice(chrono_arena.GetDevice())

while application.GetIsRunning():
    application.DoStep(1e-3)
    
    

application.EndScene()