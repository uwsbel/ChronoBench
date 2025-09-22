import chrono as ch
import chrono.irrlicht as irr
import numpy as np
import random


ch.ChEngine.Initialize()


app = irr.ChIrrApp()


system = ch.ChSystemNSC()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetEmissionRate(100)  
        self.SetInitialMass(1e-3)  

    def EmitParticle(self, time):
        particle = ch.ChParticle()
        particle.SetMass(self.GetInitialMass())

        
        particle.SetPos(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

        
        particle.SetVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

        
        quat = ch.ChQuaterniond()
        quat.SetFromRandomAngles(ch.ChVector3d(random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi)))
        particle.SetRot(quat)

        return particle

emitter = ParticleEmitter()
system.Add(emitter)


vis = irr.ChIrrVisualSystem(system)


class CustomGravitationalForce(ch.ChForce):
    def __init__(self, body):
        super(CustomGravitationalForce, self).__init__()
        self.body = body

    def GetForce(self, time):
        pos = self.body.GetPos()
        dist = np.linalg.norm(pos)
        if dist > 0:
            force = -0.1 * pos / dist  
        else:
            force = ch.ChVector3d(0, 0, 0)
        return force


for _ in range(10):  
    particle = ch.ChParticle()
    particle.SetMass(1e-3)
    particle.SetPos(ch.ChVector3d(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5)))
    particle.SetVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    system.Add(particle)

    
    force = CustomGravitationalForce(particle)
    system.Add(force)


vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Gravitational Attraction Simulation')
vis.EnableShadows()
app.SetShowFPS(True)

while vis.Run():
    system.DoStepDynamics(1e-3)