import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.ChSystemNSC()
system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "Particle Emitter Simulation", chronoirr.dimension2du(800, 600))
application.SetSkyBox()
application.AddTypicalLights()
application.SetCamera(chrono.ChVectorD(0, 0, 10))


num_particles = 100
particle_radius = 0.1
particle_mass = 1.0
gravitational_constant = 1.0


particles = []


def create_random_particle():
    
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    
    particle = chrono.ChBody()
    particle.SetMass(particle_mass)
    particle.SetPos(position)
    particle.SetPos_dt(velocity)
    particle.GetCollisionModel().ClearModel()
    particle.GetCollisionModel().AddSphere(particle_radius)
    particle.GetCollisionModel().BuildModel()
    particle.SetCollide(true)
    system.Add(particle)
    return particle


for _ in range(num_particles):
    particle = create_random_particle()
    particles.append(particle)


def apply_gravitational_forces():
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            pos_i = particles[i].GetPos()
            pos_j = particles[j].GetPos()
            distance = (pos_j - pos_i).Length()
            if distance > 0:
                force_magnitude = gravitational_constant * (particle_mass ** 2) / (distance ** 2)
                direction = (pos_j - pos_i).GetNormalized()
                force = direction * force_magnitude
                particles[i].AccumulateForce(-force)  
                particles[j].AccumulateForce(force)    


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    apply_gravitational_forces()
    
    
    system.DoStepDynamics(0.01)
    
    application.EndScene()


application.GetDevice().drop()