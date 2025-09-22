import os
import math
import numpy as np
import chrono as chrono
from chrono import ChVector3d
from chronoirr import ChVisualSystemIrrlicht, ChVisualShapeBox, ChVisualShapeSphere
from pychrono.robots import *
from pychrono.vehicle import *
from pychrono.irrlicht import *
from pychrono.contact import *
from pychrono.systems import *


sys = chrono.ChSystemNSC()
chrono.SetChronoDataPath('')  


vis = ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gravitational Particles Simulation')
vis.Initialize()
vis.AddSkyBox()
cam_pos = ChVector3d(0, 5, 10)
vis.AddCamera(cam_pos)


num_particles = 100
particle_mass = 1.0
particle_size = 0.1
particle_color = ChColor(1, 0.5, 0)  


class ParticleEmitter:
    def __init__(self, system, particle_count, particle_mass, particle_size, particle_color):
        self.system = system
        self.particles = []
        self.particle_count = particle_count
        self.particle_mass = particle_mass
        self.particle_size = particle_size
        self.particle_color = particle_color
        
    def Emit(self):
        for _ in range(self.particle_count):
            
            pos = ChVector3d(
                np.random.uniform(-10, 10),
                np.random.uniform(-10, 10),
                np.random.uniform(-10, 10)
            )
            
            
            vel = ChVector3d(
                np.random.uniform(-5, 5),
                np.random.uniform(-5, 5),
                np.random.uniform(-5, 5)
            )
            
            
            rot = ChQuaterniond(
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1)
            ).Normalize()
            
            
            if np.random.choice([0, 1]):
                shape = ChVisualShapeSphere(particle_size)
            else:
                shape = ChVisualShapeBox(particle_size)
            
            
            particle = ChBody()
            particle.SetMass(particle_mass)
            particle.SetPos(pos)
            particle.SetInertiaXX(ChVector3d(0, 0, 0))  
            particle.EnableCollision(True)  
            particle.AddVisualShape(shape)
            particle.SetColor(particle_color)
            
            
            self.system.Add(particle)
            vis.Add(particle)
            self.particles.append(particle)

    def ApplyForce(self, particles, positions, velocities):
        
        G = 100.0  
        for i in range(len(particles)):
            for j in range(i+1, len(particles)):
                p1 = particles[i]
                p2 = particles[j]
                pos1 = positions[i]
                pos2 = positions[j]
                r = pos1 - pos2
                r_length = r.Length()
                if r_length < 1e-3:
                    continue  
                force = G * p1.GetMass() * p2.GetMass() / (r_length**2)
                force_vec = r.Normalize() * force
                p1.AddForce(force_vec)
                p2.AddForce(-force_vec)  
                
        return velocities


particle_emitter = ParticleEmitter(sys, num_particles, particle_mass, particle_size, particle_color)
sys.GetContactContainer().RegisterAddContactCallback(particle_emitter)


collision_model = sys.GetContactContainer().GetDefaultCollisionModel()
collision_model.SetDefaultSuggestedEnvelope(0.01)
collision_model.SetDefaultSuggestedMargin(0.005)


time_step = 0.01
particles = []
vis.Show()

while True:
    
    particle_emitter.Emit()
    
    
    positions = [p.GetPos() for p in particles]
    velocities = [p.GetVelocity() for p in particles]
    forces = particle_emitter.ApplyForce(particles, positions, velocities)
    
    
    for i in range(len(particles)):
        particle = particles[i]
        particleem = particle.GetInertiaXX()
        particle.AddForce(forces[i])
        particle.UpdateDynamics()
        particle.UpdateCollision()
        particle.UpdateVisual()
    
    
    for i in range(len(particles)):
        if positions[i].Y < -5:
            del particles[i]
            vis.Remove(particles[i])
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    if vis.Run():
        continue
    else:
        break


vis.Destroy()
sys.Destroy()