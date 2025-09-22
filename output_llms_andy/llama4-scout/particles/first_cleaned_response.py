import chrono as ch
import chrono.irrlicht as chirr
import numpy as np
import random


ch.ChEngine.Initialize()


app = chirr.ChIrrApp()


system = ch.ChSystemNSC()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetEmissionRate(100)  
        self.SetParticleMass(1e-3)  
        self.SetParticleRadius(0.01)  

    def EmitParticles(self, time):
        for _ in range(int(self.GetEmissionRate())):
            
            pos = ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

            
            vel = ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

            
            quat = ch.ChQuaterniond()
            quat.SetFromRandom()

            
            particle = ch.ChParticle()
            particle.SetPos(pos)
            particle.SetPos_dt(vel)
            particle.SetRot(quat)
            particle.SetMass(self.GetParticleMass())
            particle.SetRadius(self.GetParticleRadius())

            
            system.Add(particle)

emitter = ParticleEmitter()
system.Add(emitter)


vis = chirr.ChIrrlichtVisualizer()
vis.AttachSystem(system)
vis.SetWindowSize(chirr.ChVector2i(800, 600))
vis.SetWindowTitle('Gravitational Attraction Simulation')
app.SetVisualizer(vis)


while vis.Run() and not app.GetQuitRequestFlag():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)