import chrono as ch
import chrono.irrlicht as chirr
import numpy as np
import random

# Initialize PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = chirr.ChIrrApp()

# Create a Chrono system
system = ch.ChSystemNSC()

# Set gravity
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))

# Create a particle emitter
class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetEmissionRate(100)  # particles per second
        self.SetParticleMass(1e-3)  # mass in kg
        self.SetParticleRadius(0.01)  # radius in meters

    def EmitParticles(self, time):
        for _ in range(int(self.GetEmissionRate())):
            # Random position near the origin
            pos = ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

            # Random velocity
            vel = ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

            # Random orientation
            quat = ch.ChQuaterniond()
            quat.SetFromRandom()

            # Create a new particle
            particle = ch.ChParticle()
            particle.SetPos(pos)
            particle.SetPos_dt(vel)
            particle.SetRot(quat)
            particle.SetMass(self.GetParticleMass())
            particle.SetRadius(self.GetParticleRadius())

            # Add particle to the system
            system.Add(particle)

emitter = ParticleEmitter()
system.Add(emitter)

# Create Irrlicht scene node for visualization
vis = chirr.ChIrrlichtVisualizer()
vis.AttachSystem(system)
vis.SetWindowSize(chirr.ChVector2i(800, 600))
vis.SetWindowTitle('Gravitational Attraction Simulation')
app.SetVisualizer(vis)

# Simulation loop
while vis.Run() and not app.GetQuitRequestFlag():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    system.DoStepDynamics(1e-3)  # 1ms time step