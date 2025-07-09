import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr

# Initialize PyChrono environment
chrono.SetChronoDataPath('/path/to/data')

# Create a Chrono system
sys = chrono.ChSystemSMC()

# Set gravity
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))

# Create a visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Gravitational Attraction Simulation')
vis.Initialize()

# Create a particle emitter
class ParticleEmitter:
    def __init__(self, sys, rate, position):
        self.sys = sys
        self.rate = rate
        self.position = position

    def EmitParticles(self):
        for _ in range(self.rate):
            # Random particle properties
            radius = np.random.uniform(0.1, 1.0)
            mass = np.random.uniform(1.0, 10.0)
            velocity = chrono.ChVector3d(np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0))
            orientation = chrono.QuatFromAngleAxis(np.random.uniform(0, 2 * math.pi), chrono.ChVector3d(0, 0, 1))

            # Create a sphere body
            body = chrono.ChBodyEasySphere(radius, mass, True, True, chrono.ChContactMaterialSMC())
            body.SetPos(self.position + chrono.ChVector3d(np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0), np.random.uniform(-1.0, 1.0)))
            body.SetRot(orientation)
            body.SetLinVel(velocity)
            self.sys.Add(body)

# Initialize particle emitter
emitter = ParticleEmitter(sys, 10, chrono.ChVector3d(0, 0, 0))

# Simulation loop
while vis.Run():
    # Emit particles
    emitter.EmitParticles()

    # Advance simulation
    sys.DoStepDynamics(0.01)

    # Clear forces (not necessary in this case, but useful for other simulations)
    # sys.ClearForces()

    # Render visualization
    vis.BeginScene()
    vis.Render()
    vis.EndScene()