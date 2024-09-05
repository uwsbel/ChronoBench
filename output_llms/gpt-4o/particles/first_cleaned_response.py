import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import numpy as np


chrono.SetChronoDataPath('path_to_chrono_data')


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, 0))


application = chronoirr.ChIrrApp(system, 'Particle Attraction Simulation', chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLogo()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, -10))
application.AddLightWithShadow(chronoirr.vector3df(10, 10, 10), chronoirr.vector3df(0, 0, 0), 9, 1, 9, 40)


def create_random_particle(system):
    shape_type = random.choice(['sphere', 'box'])
    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.5)
        particle = chrono.ChBodyEasySphere(radius, 1000, True, True)
    else:
        size_x = random.uniform(0.1, 0.5)
        size_y = random.uniform(0.1, 0.5)
        size_z = random.uniform(0.1, 0.5)
        particle = chrono.ChBodyEasyBox(size_x, size_y, size_z, 1000, True, True)
    
    
    particle.SetPos(chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5)))
    
    
    particle.SetPos_dt(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
    
    
    particle.SetRot(chrono.ChQuaternionD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)).GetNormalized())
    
    
    system.Add(particle)


for _ in range(10):
    create_random_particle(system)


def apply_gravitational_forces(system):
    G = 6.67430e-11  
    bodies = system.Get_bodylist()
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            body1 = bodies[i]
            body2 = bodies[j]
            pos1 = body1.GetPos()
            pos2 = body2.GetPos()
            r = pos2 - pos1
            distance = r.Length()
            if distance > 1e-5:
                force_magnitude = G * body1.GetMass() * body2.GetMass() / (distance ** 2)
                force = r.GetNormalized() * force_magnitude
                body1.Accumulate_force(force, pos1, False)
                body2.Accumulate_force(-force, pos2, False)


application.SetTimestep(0.01)
while application.GetDevice().run():
    
    apply_gravitational_forces(system)
    
    
    if random.random() < 0.1:
        create_random_particle(system)
    
    
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()