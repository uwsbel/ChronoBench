import chrono as ch
import chrono.irrlicht as chirr
import numpy as np
import random


ch.ChEngine.Initialize()


app = chirr.ChIrrApp()


system = ch.ChSystem()


system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))


class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetEmissionRate(100)  
        self.SetInitialShape(ch.ChParticleEmitter.Shape.SPHERE)
        self.SetInitialSizeDistribution(ch.ChParticleEmitter.SizeDistribution.UNIFORM, 0.1, 0.1)
        self.SetInitialVelocityDistribution(ch.ChParticleEmitter.VelocityDistribution.GAUSSIAN, ch.ChVector3d(0, 0, 0), ch.ChVector3d(1, 1, 1))
        self.SetInitialPositionDistribution(ch.ChParticleEmitter.PositionDistribution.UNIFORM, ch.ChVector3d(0, 10, 0), ch.ChVector3d(10, 10, 10))

    def Update(self, step):
        if random.random() < 0.1:  
            pos = ch.ChVector3d(random.uniform(-5, 5), random.uniform(5, 15), random.uniform(-5, 5))
            vel = ch.ChVector3d(random.uniform(-1, 1), random.uniform(-5, 0), random.uniform(-1, 1))
            self.EmitParticles(pos, vel, ch.ChQuaterniond(1, 0, 0, 0), 0.1)

emitter = ParticleEmitter()
system.Add(emitter)


vis = chirr.ChIrrSceneNode(system.GetVisualSystem())
app.SetSceneNode(vis)


class CustomGravityCallback(ch.ChCustomIntegratorCallback):
    def __init__(self, system):
        super(CustomGravityCallback, self).__init__()
        self.system = system

    def ComputeForces(self, bodies, forces):
        G = 0.1  
        for body in bodies:
            if isinstance(body, ch.ChBody):
                pos = body.GetPos()
                for other_body in bodies:
                    if other_body != body and isinstance(other_body, ch.ChBody):
                        other_pos = other_body.GetPos()
                        distance = (pos - other_pos).Length()
                        if distance > 0:
                            force = G * body.GetMass() * other_body.GetMass() / (distance ** 2) * (other_pos - pos).Normalized()
                            forces[body].AddForce(force, pos)

callback = CustomGravityCallback(system)
system.SetCustomIntegratorCallback(callback)


app.Initialize()
app.SetSimulationRate(1 / 60)
while app.Run():
    system.DoStepDynamics(1 / 60)