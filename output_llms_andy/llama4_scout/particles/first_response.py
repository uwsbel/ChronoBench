import chrono as ch
import chrono.irrlicht as irr
import numpy as np
import random

# Initialize PyChrono environment
ch.ChEngine.Initialize()

# Create the Irrlicht application
app = irr.ChIrrApp()

# Create a Chrono system
system = ch.ChSystemNSC()

# Set gravity
system.SetGravitationalAcceleration(ch.ChVector3d(0, -9.81, 0))

# Create a particle emitter
class ParticleEmitter(ch.ChParticleEmitter):
    def __init__(self):
        super(ParticleEmitter, self).__init__()
        self.SetEmissionRate(100)  # particles per second
        self.SetInitialMass(1e-3)  # mass of emitted particles

    def EmitParticle(self, time):
        particle = ch.ChParticle()
        particle.SetMass(self.GetInitialMass())

        # Random position near origin
        particle.SetPos(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

        # Random velocity
        particle.SetVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))

        # Random orientation
        quat = ch.ChQuaterniond()
        quat.SetFromRandomAngles(ch.ChVector3d(random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi), random.uniform(0, 2*np.pi)))
        particle.SetRot(quat)

        return particle

emitter = ParticleEmitter()
system.Add(emitter)

# Create Irrlicht scene node for visualization
vis = irr.ChIrrVisualSystem(system)

# Add custom gravitational force (simple attraction to origin)
class CustomGravitationalForce(ch.ChForce):
    def __init__(self, body):
        super(CustomGravitationalForce, self).__init__()
        self.body = body

    def GetForce(self, time):
        pos = self.body.GetPos()
        dist = np.linalg.norm(pos)
        if dist > 0:
            force = -0.1 * pos / dist  # Adjust coefficient as needed
        else:
            force = ch.ChVector3d(0, 0, 0)
        return force

# Add particles and custom force
for _ in range(10):  # Initial particles
    particle = ch.ChParticle()
    particle.SetMass(1e-3)
    particle.SetPos(ch.ChVector3d(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5)))
    particle.SetVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    system.Add(particle)

    # Apply custom gravitational force to each particle
    force = CustomGravitationalForce(particle)
    system.Add(force)

# Setup and start simulation
vis.AttachSystem(system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Gravitational Attraction Simulation')
vis.EnableShadows()
app.SetShowFPS(True)

while vis.Run():
    system.DoStepDynamics(1e-3)