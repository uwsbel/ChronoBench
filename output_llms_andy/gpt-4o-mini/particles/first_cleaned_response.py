import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import random
import numpy as np


chrono.ChSystemNSC()
system = chrono.ChSystemNSC()


application = chronoirr.ChIrrApp(system, "Particle Gravitational Attraction", chronoirr.dimension2du(800, 600))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 0, 10))
application.AddLight(chrono.ChVectorD(0, 10, 0))


num_particles = 100
particles = []
particle_shapes = [chrono.ChSphere(0.1), chrono.ChBox(0.1, 0.1, 0.1), chrono.ChCylinder(0.1, 0.2)]
gravitational_constant = 1.0


for i in range(num_particles):
    
    shape = random.choice(particle_shapes)
    
    
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    
    
    particle = chrono.ChBody()
    particle.SetMass(1.0)
    particle.SetPos(position)
    particle.SetPos_dt(velocity)
    particle.GetCollisionModel().ClearModel()
    particle.GetCollisionModel().AddSphere(0.1)
    particle.GetCollisionModel().BuildModel()
    particle.SetCollide(true)
    
    
    system.Add(particle)
    particles.append(particle)


def apply_gravitational_forces():
    for i, particle in enumerate(particles):
        force = chrono.ChVectorD(0, 0, 0)
        for j, other_particle in enumerate(particles):
            if i != j:
                r = other_particle.GetPos() - particle.GetPos()
                distance = r.Length()
                if distance > 0:
                    force += (gravitational_constant * particle.GetMass() * other_particle.GetMass() / distance**2) * r.Normalize()
        particle.SetForce(force)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    apply_gravitational_forces()

    
    system.DoStepDynamics(0.01)

    
    application.EndScene()