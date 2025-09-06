import os
import math
import numpy as np
import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath('/path/to/data')


my_system = chrono.ChSystemSMC()
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gravitational Particle Simulation')
vis.Initialize()


def gravity_function(radius):
    return -6.67430e-11 * 5.972e24 / (radius ** 2)


class ParticleEmitter:
    def __init__(self, system, position, radius, max_particles):
        self.system = system
        self.position = position
        self.radius = radius
        self.max_particles = max_particles
        self.particles = []

    def emit_particle(self):
        if len(self.particles) < self.max_particles:
            particle = chrono.ChBody()
            particle.SetMass(1)
            particle.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
            particle.SetPos(self.position + chrono.ChVector3d(np.random.uniform(-self.radius, self.radius), np.random.uniform(-self.radius, self.radius), np.random.uniform(-self.radius, self.radius)))
            particle.SetRot(chrono.ChQuaternionD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)).Normalize())
            particle.SetPos_dt(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            particle.SetWvel_par(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)))
            particle.SetBuoyancyForce(gravity_function(particle.GetPos().Distance(self.position)))
            self.system.Add(particle)
            self.particles.append(particle)


emitter = ParticleEmitter(my_system, chrono.ChVector3d(0, 0, 0), 10, 1000)
for _ in range(1000):
    emitter.emit_particle()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics(0.01)

vis.GetDevice().closeDevice()