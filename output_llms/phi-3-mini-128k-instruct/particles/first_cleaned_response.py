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
    return chrono_arena.GetGravityAcceleration(acceleration)


particle_system.SetGravityFunction(custom_gravitational_force)


default_particle_parameters = {
    'shape': ChronoVector3d(1.0, 1.0, 1.0),  
    'velocity': ChronoVector3d(0.0, 0.0, 0.0),  
    'position': ChronoVector3d(0.0, 0.0, 10.0),  
    'orientation': ChronoQuaterniond(1.0, 0.0, 0.0, 0.0)  
}


for _ in range(100):  
    shape = particle_system.AddParticle(
        default_particle_parameters['shape'],
        default_particle_parameters['position'],
        default_particle_parameters['velocity'],
        default_particle_parameters['orientation']
    )


while application.IsRunning():
    application.DoStep(chrono_arena, 1e-3, 1)

    
    application.BeginScene()
    application.DrawAll()
    application.RenderState().SetCameraHorizontalFOV(45.0)
    application.RenderAll()
    application.EndScene()