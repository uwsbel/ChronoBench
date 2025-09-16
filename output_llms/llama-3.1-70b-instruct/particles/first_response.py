import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Create the simulation system and set the length of the time step
mysystem = chrono.ChSystemNSC()
mysystem.SetStep(0.01)

# Create the Irrlicht visualization (open the Irrlicht window)
myapplication = chronoirr.ChVisualSystemIrrlicht(mysystem)
myapplication.SetWindowSize(800, 600)
myapplication.SetWindowTitle("Gravitational Attraction")
myapplication.SetCamera(chrono.ChVectorD(0, 0, 10), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0))

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def emit_particles(self):
        for _ in range(self.num_particles):
            # Create a particle with random shape, position, velocity, and orientation
            particle = chrono.ChBodyEasySphere(self.system, 0.1, 1000, chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            particle.SetPos(chrono.ChVectorD(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(-5, 5)))
            particle.SetVel(chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            particle.SetRot(chrono.ChQuaternionD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            self.particles.append(particle)
            self.system.Add(particle)

    def apply_gravitational_force(self):
        for i in range(len(self.particles)):
            for j in range(i + 1, len(self.particles)):
                particle1 = self.particles[i]
                particle2 = self.particles[j]
                distance = (particle2.GetPos() - particle1.GetPos()).Length()
                force = 0.1 / (distance ** 2)
                direction = (particle2.GetPos() - particle1.GetPos()).GetNormalized()
                particle1.ApplyForceToCOF(force * direction)
                particle2.ApplyForceToCOF(-force * direction)

emitter = ParticleEmitter(mysystem, 100)
emitter.emit_particles()

# Simulation loop
while myapplication.Run():
    mysystem.DoStepDynamics(0.01)
    emitter.apply_gravitational_force()
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.EndScene()

# Delete the visualization application when the simulation ends
del myapplication