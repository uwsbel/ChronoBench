import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


system = chrono.ChSystemNSC()


system.Set_G_acc(chrono.ChVectorD(0, 0, 0))


application = chronoirr.ChIrrApp(system, "Particle Gravitational Attraction", chronoirr.dimension2du(1024, 768))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 0, 30))


emission_rate = 5  
max_particles = 100
particle_mass = 1.0
particle_radius = 0.2
emission_area = 10.0


G = 6.67430e-11 * 1e9  

particles = []


class GravitationalForce(chrono.PyChForce):
    def __init__(self, particles):
        super().__init__()
        self.particles = particles

    def Force(self, time, body):
        force = chrono.ChVectorD(0, 0, 0)
        for other in self.particles:
            if other != body:
                r = body.GetPos() - other.GetPos()
                distance = r.Length()
                if distance > 0:
                    magnitude = G * body.GetMass() * other.GetMass() / (distance * distance)
                    force += -magnitude * r.GetNormalized()
        return force


def emit_particle():
    if len(particles) < max_particles:
        
        x = random.uniform(-emission_area/2, emission_area/2)
        y = random.uniform(-emission_area/2, emission_area/2)
        z = random.uniform(-emission_area/2, emission_area/2)

        
        vx = random.uniform(-1, 1)
        vy = random.uniform(-1, 1)
        vz = random.uniform(-1, 1)

        
        body = chrono.ChBodyEasySphere(particle_radius, 1000)
        body.SetPos(chrono.ChVectorD(x, y, z))
        body.SetPos_dt(chrono.ChVectorD(vx, vy, vz))
        body.SetMass(particle_mass)

        
        col = chrono.ChColorAsset()
        col.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        body.AddAsset(col)

        
        grav_force = GravitationalForce(particles)
        body.AddForce(grav_force)

        system.Add(body)
        particles.append(body)


application.SetTimestep(0.01)
application.AssetBindAll()
application.AssetUpdateAll()

time_counter = 0

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()

    
    time_counter += application.GetTimestep()
    if time_counter >= 1.0 / emission_rate:
        emit_particle()
        time_counter = 0

    application.DoStep()
    application.EndScene()