import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


mysystem = chrono.ChSystemNSC()


mysystem.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))


class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def emit_particles(self):
        for _ in range(self.num_particles):
            
            shape_type = np.random.choice(['sphere', 'box'])

            if shape_type == 'sphere':
                
                radius = np.random.uniform(0.1, 0.5)
                
                particle = chrono.ChBodyEasySphere(self.system, radius, 1000, True)
            else:
                
                length = np.random.uniform(0.1, 0.5)
                width = np.random.uniform(0.1, 0.5)
                height = np.random.uniform(0.1, 0.5)
                
                particle = chrono.ChBodyEasyBox(self.system, length, width, height, 1000, True)

            
            position = chrono.ChVectorD(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(-5, 5))
            particle.SetPos(position)

            
            velocity = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
            particle.SetVel(velocity)

            
            orientation = chrono.Q_from_AngX(np.random.uniform(-np.pi, np.pi))
            particle.SetRot(orientation)

            
            self.system.Add(particle)
            self.particles.append(particle)


emitter = ParticleEmitter(mysystem, 10)
emitter.emit_particles()


class CustomGravitationalForce(chrono.ChForce):
    def __init__(self, particle1, particle2):
        super().__init__()
        self.particle1 = particle1
        self.particle2 = particle2

    def ComputeF(self, time):
        
        distance = (self.particle2.GetPos() - self.particle1.GetPos()).Length()

        
        force = 6.67430e-11 * self.particle1.GetMass() * self.particle2.GetMass() / (distance ** 2)

        
        self.particle1.ApplyForceToCOF(chrono.ChForce(chrono.ChVectorD(0, 0, force)))
        self.particle2.ApplyForceToCOF(chrono.ChForce(chrono.ChVectorD(0, 0, -force)))


for i in range(len(emitter.particles)):
    for j in range(i + 1, len(emitter.particles)):
        force = CustomGravitationalForce(emitter.particles[i], emitter.particles[j])
        mysystem.AddForce(force)


myapplication = chronoirr.ChVisualSystemIrrlicht(mysystem)


myapplication.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0))


mysystem.SetStep(0.01)
mysystem.SetMaxTime(10)


while mysystem.GetChTime() < mysystem.GetMaxTime():
    mysystem.DoStep()
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()