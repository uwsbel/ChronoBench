import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random

# Initialize the PyChrono system
chrono.SetChronoDataPath('/path/to/data')  # Update with your data path
my_system = chrono.ChSystemNSC()

# Visualization setup
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravitational Attraction Simulation')
vis.Initialize()
vis.AddTypicalLights()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 10, 20))

# Parameters
num_particles = 100
particle_radius = 0.5
gravitational_constant = 1.0
time_step = 0.01
max_velocity = 5.0

# Particle class
class Particle:
    def __init__(self, position, velocity):
        self.body = chrono.ChBodyEasySphere(particle_radius, 1000, True, True)
        self.body.SetPos(position)
        self.body.SetLinVel(velocity)
        self.body.SetCollide(True)
        my_system.Add(self.body)

    def apply_gravity(self, particles):
        for other in particles:
            if other != self:
                distance_vector = other.body.GetPos() - self.body.GetPos()
                distance = distance_vector.Length()
                if distance > 0:
                    force_magnitude = gravitational_constant / (distance ** 2)
                    force = distance_vector.GetNormalized() * force_magnitude
                    self.body.SetForce(force)

# Particle emitter
def emit_particles():
    particles = []
    for _ in range(num_particles):
        position = chrono.ChVector3d(random.uniform(-10, 10), random.uniform(5, 15), random.uniform(-10, 10))
        velocity = chrono.ChVector3d(random.uniform(-max_velocity, max_velocity), random.uniform(-max_velocity, max_velocity), random.uniform(-max_velocity, max_velocity))
        particle = Particle(position, velocity)
        particles.append(particle)
    return particles

# Create particles
particles = emit_particles()

# Simulation loop
while vis.Run():
    # Begin the scene
    vis.BeginScene()
    vis.Render()

    # Apply gravitational forces
    for particle in particles:
        particle.apply_gravity(particles)

    # Step the dynamics
    my_system.DoStepDynamics(time_step)

    # End the scene
    vis.EndScene()

# Clean up
vis.Close()