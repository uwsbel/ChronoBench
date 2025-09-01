import pychrono.core as chrono
import pychrono.irr as chrono_irr
import random
import math


chrono.ChSystemNSC()


app = chrono_irr.ChIrrApp(chrono.ChSystemNSC(), "Particle Emitter Simulation", chrono.irr.dimension2d(800, 600))
app.AddLogo()
app.AddSkyBox()
app.AddCamera(chrono.irr.vector3df(0, 5, -15))
app.AddLight(chrono.irr.vector3df(0, 10, -10))


num_particles = 100
particle_radius = 0.1
gravity_constant = 6.67430e-11  
time_step = 0.01
emission_rate = 5  


particles = []


def create_random_particle():
    
    position = chrono.ChVectorD(random.uniform(-10, 10), random.uniform(0, 10), random.uniform(-10, 10))
    
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    
    particle = chrono.ChBody()
    particle.SetMass(1.0)  
    particle.SetPos(position)
    particle.SetPos_dt(velocity)
    particle.GetCollisionModel().ClearModel()
    particle.GetCollisionModel().AddSphere(particle_radius)
    particle.GetCollisionModel().BuildModel()
    particle.SetCollide(True)
    chrono.ChCollisionSystemNSC().AddBody(particle)
    return particle


for _ in range(num_particles):
    particle = create_random_particle()
    particles.append(particle)
    chrono.ChSystemNSC().Add(particle)


while app.GetDevice().run():
    app.BeginScene()
    app.DrawAll()

    
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            pos_i = particles[i].GetPos()
            pos_j = particles[j].GetPos()
            distance = (pos_j - pos_i).Length()
            if distance > 0:
                force_magnitude = gravity_constant * (1.0 * 1.0) / (distance * distance)  
                force_direction = (pos_j - pos_i).GetNormalized()
                force = force_direction * force_magnitude
                particles[i].AccumulateForce(force)
                particles[j].AccumulateForce(-force)

    
    chrono.ChSystemNSC().DoStepDynamics(time_step)

    
    if random.random() < (emission_rate * time_step):
        new_particle = create_random_particle()
        particles.append(new_particle)
        chrono.ChSystemNSC().Add(new_particle)

    app.EndScene()


app.GetDevice().drop()