import os
import math
import numpy as np
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), "data"))


sys = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()


vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Gravitational Attraction Simulation")
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles
        self.particles = []

    def emit_particle(self):
        
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
            
            force = chrono.ChVector3d(0, 0, -9.81)
            particle.ApplyForce(force)


emitter = ParticleEmitter(sys, 100)


for _ in range(emitter.num_particles):
    emitter.emit_particle()


while vis.Run():
    vis.BeginScene()
    vis.DrawAll()

    
    emitter.update_particles()

    
    sys.DoStepDynamics(0.01)

    vis.EndScene()


sys.RemoveAll()


vis.GetSystem().Exit()