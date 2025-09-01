import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


chrono.SetChronoDataPath('path/to/chrono/data')
my_system = chrono.ChSystemNSC()


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(my_system)
vis.SetWindowSize(800, 600)
vis.SetWindowTitle('Particle Gravitational Attraction Simulation')
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 10, -20), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


def gravitational_force(particle1, particle2):
    G = 6.67430e-11  
    m1 = particle1.GetMass()
    m2 = particle2.GetMass()
    pos1 = particle1.GetPos()
    pos2 = particle2.GetPos()
    r = (pos2 - pos1).Length()
    if r < 1e-5:
        return chrono.ChVectorD(0, 0, 0)
    return G * m1 * m2 / r**2 * (pos2 - pos1).GetNormalized()


particles = []


def emit_particle():
    
    radius = random.uniform(0.1, 0.5)
    density = 1000  
    mass = (4/3) * math.pi * radius**3 * density

    
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(0, 10), random.uniform(-5, 5))

    
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))

    
    particle = chrono.ChBodyEasySphere(radius, density, True, True)
    particle.SetPos(position)
    particle.SetPos_dt(velocity)

    
    my_system.Add(particle)
    particles.append(particle)


for _ in range(10):
    emit_particle()


class CustomTimestepper(chrono.ChTimestepper):
    def Advance(self, step):
        super().Advance(step)
        
        for i, particle1 in enumerate(particles):
            force = chrono.ChVectorD(0, 0, 0)
            for j, particle2 in enumerate(particles):
                if i != j:
                    force += gravitational_force(particle1, particle2)
            particle1.Accumulate_force(force, particle1.GetPos(), False)


my_system.SetTimestepper(CustomTimestepper())


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    my_system.DoStepDynamics(0.01)

    
    if my_system.GetChTime() % 1 < 0.01:
        emit_particle()

vis.GetDevice().closeDevice()