# Import necessary libraries and modules
import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

# Set the path to the Chrono data directory
chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))

# Create the simulation system
sys = chrono.ChSystemNSC()

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()

# Set up the visualization window
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Gravitational Attraction Simulation")
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def emit_particle(self):
        # Create a new particle with random shape, position, velocity, and orientation
        particle_shape = chrono.ChVisualShapeSphere(np.random.uniform(0.1, 0.5))
        particle_pos = chrono.ChVector3d(np.random.uniform(-5, 5), np.random.uniform(-5, 5), np.random.uniform(-5, 5))
        particle_vel = chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
        particle_rot = chrono.Q_from_AngX(np.random.uniform(0, 2 * math.pi))

        particle = chrono.ChBodyEasySphere(1, 1000, True, True)
        particle.AddVisualShape(particle_shape)
        particle.SetPos(particle_pos)
        particle.SetVel(particle_vel)
        particle.SetRot(particle_rot)

        self.system.Add(particle)
        self.particles.append(particle)

    def update_particles(self):
        for particle in self.particles:
            # Apply custom gravitational force to simulate particle attraction
            force = chrono.ChVector3d(0, 0, -9.81)
            particle.ApplyForce(force)

# Create a particle emitter with 100 particles
emitter = ParticleEmitter(sys, 100)

# Emit particles
for _ in range(emitter.num_particles):
    emitter.emit_particle()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()

    # Update particles
    emitter.update_particles()

    # Advance the simulation
    sys.DoStepDynamics(0.01)

    vis.EndScene()

# Remove all objects from the simulation system
sys.RemoveAll()

# Exit the simulation
vis.GetSystem().Exit()