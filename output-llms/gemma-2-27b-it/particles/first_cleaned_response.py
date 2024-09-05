import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random


my_system = chrono.ChSystemNSC()
my_system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))  


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Attraction Simulation')
vis.Initialize()


num_particles = 100
emission_rate = 1  
particle_radius = 0.1


emitter = chrono.ChParticleEmitter(my_system)
emitter.SetPos(chrono.ChVectorD(0, 5, 0))
emitter.SetFlowRate(emission_rate)
emitter.SetParticleRadius(particle_radius)


particle_shape = chrono.ChSphereShape()
particle_material = chrono.ChMaterialSurfaceNSC()
particle_material.SetFriction(0.5)
particle_material.SetRestitution(0.2)


def create_particle():
    pos = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(0, 10), random.uniform(-5, 5))
    vel = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    rot = chrono.ChQuaternionD(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
    particle = chrono.ChBodyEasySphere(particle_radius, 1000, True, True)
    particle.SetPos(pos)
    particle.SetVel(vel)
    particle.SetRot(rot)
    particle.SetCollide(True)
    particle.GetCollisionModel().ClearModel()
    particle.GetCollisionModel().AddShape(particle_shape, particle_material)
    my_system.Add(particle)
    return particle


for _ in range(num_particles):
    create_particle()


def apply_gravity(particle1, particle2):
    G = 6.67430e-11  
    m1 = particle1.GetMass()
    m2 = particle2.GetMass()
    r = particle2.GetPos() - particle1.GetPos()
    dist = r.Length()
    force = G * m1 * m2 / (dist ** 2) * r.GetNormalized()
    particle1.AddForce(force)
    particle2.AddForce(-force)


while vis.Run():
    
    for _ in range(emission_rate):
        create_particle()

    
    for i in range(my_system.Get_bodylist().size()):
        for j in range(i + 1, my_system.Get_bodylist().size()):
            particle1 = my_system.Get_bodylist()[i]
            particle2 = my_system.Get_bodylist()[j]
            apply_gravity(particle1, particle2)

    my_system.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.Render()
    vis.EndScene()