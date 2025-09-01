from pychrono import *
from pychrono.irrlicht import *
from pychrono.particles import *
from pychrono.fea import *
import irrlicht
import random


random.seed(0)


my_system = ChronoSystem()
my_visualization = IrrlichtVisualization(my_system, "Particle Attraction Simulation")


gravity = Vector3D(0.0, -9.81, 0.0)
my_system.AddForce Law2(gravity, my_system.GetChronoData().Get_system_time())


emitter = ParticleEmitter(my_system)
emitter.SetParticleRadius(0.1)
emitter.SetParticleMass(1.0)


num_particles = 50
for i in range(num_particles):
    
    shape_type = random.choice(["SPHERE", "BOX", "CYLINDER"])
    if shape_type == "SPHERE":
        particle = Particle(my_system, ChVectorD(0, 0, 0), 1.0, ChVectorD(0, 0, 0))
    elif shape_type == "BOX":
        particle = Particle(my_system, ChVectorD(0, 0, 0), 1.0, ChVectorD(0, 0, 0))
        particle.SetBoxShape(ChVectorD(0.2, 0.2, 0.2))
    elif shape_type == "CYLINDER":
        particle = Particle(my_system, ChVectorD(0, 0, 0), 1.0, ChVectorD(0, 0, 0))
        particle.SetCylinderShape(0.1, 0.2)

    
    pos = ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    vel = ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    particle.SetPos(pos)
    particle.SetVel(vel)
    particle.SetWvel(ChVectorD(0, 0, 0))

    
    emitter.AddParticle(particle)


for i in range(num_particles):
    for j in range(i + 1, num_particles):
        dist = emitter.GetParticle(i).GetPos() - emitter.GetParticle(j).GetPos()
        force_mag = 100.0 / dist.Length()
        force = dist.Normalize() * force_mag
        my_system.AddForce(Law2(force, my_system.GetChronoData().Get_system_time()), emitter.GetParticle(i))
        my_system.AddForce(Law2(-force, my_system.GetChronoData().Get_system_time()), emitter.GetParticle(j))


my_system.Set_delta_time(1e-3)
sim_time = 10.0
while my_system.GetChronoData().Get_system_time() < sim_time:
    my_system.DoStepDynamics()
    my_visualization.Run()


my_visualization.Wait()
my_visualization.Close()