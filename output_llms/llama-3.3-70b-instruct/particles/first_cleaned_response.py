import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as chronopardisomkl
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetDampingF(0.2)
contact_material.SetCompliance(0.0001)
contact_material.SetComplianceT(0.0001)


class ParticleEmitter:
    def __init__(self, num_particles):
        self.num_particles = num_particles
        self.particles = []

    def emit_particles(self):
        for _ in range(self.num_particles):
            
            shape = np.random.choice(['sphere', 'box', 'cylinder'])
            position = np.random.uniform(-10, 10, 3)
            velocity = np.random.uniform(-1, 1, 3)
            orientation = np.random.uniform(0, np.pi, 3)

            
            body = chrono.ChBodyEasySphere(sys, 0.1, 1000, contact_material)
            body.SetPos(chrono.ChVectorD(*position))
            body.SetLinearVel(chrono.ChVectorD(*velocity))
            body.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(*orientation), 1))

            
            sys.Add(body)
            self.particles.append(body)


emitter = ParticleEmitter(100)
emitter.emit_particles()


def apply_gravitational_forces():
    for i in range(len(emitter.particles)):
        for j in range(i + 1, len(emitter.particles)):
            particle1 = emitter.particles[i]
            particle2 = emitter.particles[j]

            
            distance = (particle2.GetPos() - particle1.GetPos()).Length()

            
            force = 0.1 * particle1.GetMass() * particle2.GetMass() / (distance ** 2)

            
            direction = (particle2.GetPos() - particle1.GetPos()).GetNormalized()
            particle1.ApplyForce(chrono.ChForce(direction * force))
            particle2.ApplyForce(chrono.ChForce(-direction * force))


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("PyChrono Simulation")


while vis.Run():
    
    apply_gravitational_forces()

    
    sys.DoStepDynamics(0.01)

    
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()