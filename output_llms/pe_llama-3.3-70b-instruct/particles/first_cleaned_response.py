import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('./')


sys = chrono.ChSystemNSC()


contact_material = chrono.ChContactMaterial()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.1)
contact_material.SetCompliance(0.01)
sys.SetDefaultContactMaterial(contact_material)


class ParticleEmitter:
    def __init__(self, sys):
        self.sys = sys
        self.particles = []

    def emit_particle(self):
        
        pos = chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-10, 10))
        vel = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
        rot = chrono.Q_from_AngXAxis(np.random.uniform(0, 2 * math.pi), chrono.ChVectorD(0, 0, 1))

        
        body = chrono.ChBodyEasySphere(0.1, 1000, True, True, contact_material)
        body.SetPos(pos)
        body.SetRot(rot)
        body.SetVel(vel)

        
        self.sys.Add(body)
        self.particles.append(body)

    def apply_gravity(self):
        
        for i, particle in enumerate(self.particles):
            for j in range(i + 1, len(self.particles)):
                other_particle = self.particles[j]
                distance = (particle.GetPos() - other_particle.GetPos()).Length()
                force = 0.1 * particle.GetMass() * other_particle.GetMass() / (distance ** 2)
                direction = (other_particle.GetPos() - particle.GetPos()).Normalized()
                particle.ApplyForceToCenter(force * direction)


emitter = ParticleEmitter(sys)
for _ in range(100):
    emitter.emit_particle()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Attraction Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3D(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()

    
    emitter.apply_gravity()
    sys.DoStepDynamics(0.01)