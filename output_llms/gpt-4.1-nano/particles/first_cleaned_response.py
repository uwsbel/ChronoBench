import pychrono.core as chrono
import pychrono.animate as animate
import random
import math


chrono.SetChronoDataPath('/path/to/data/')  
system = chrono.ChSystemNSC()


gravity = chrono.ChVectorD(0, -9.81, 0)
system.Set_G_acc(gravity)


ground = chrono.ChBodyEasyBox(10, 1, 10, 1000, True, True)
ground.SetPos(chrono.ChVectorD(0, -0.5, 0))
system.Add(ground)


num_particles = 50  
emission_area_center = chrono.ChVectorD(0, 2, 0)
emission_area_size = 1.0  


particles = []


def create_random_shape():
    shape_type = random.choice(['sphere', 'box'])
    density = 1000  
    if shape_type == 'sphere':
        radius = random.uniform(0.05, 0.2)
        shape = chrono.ChBodyEasySphere(radius, density, True, True)
    else:
        size = [random.uniform(0.05, 0.2) for _ in range(3)]
        shape = chrono.ChBodyEasyBox(size[0], size[1], size[2], density, True, True)
    return shape


def generate_particle():
    body = create_random_shape()
    
    pos = chrono.ChVectorD(
        emission_area_center.x + random.uniform(-emission_area_size/2, emission_area_size/2),
        emission_area_center.y + random.uniform(-emission_area_size/2, emission_area_size/2),
        emission_area_center.z + random.uniform(-emission_area_size/2, emission_area_size/2)
    )
    body.SetPos(pos)
    
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(0, 2),
        random.uniform(-1, 1)
    )
    body.SetPos(pos)
    body.SetRot(chrono.Q_from_EulerAngles(
        random.uniform(0, math.pi),
        random.uniform(0, math.pi),
        random.uniform(0, math.pi)
    ))
    body.SetPos(pos)
    body.SetMassProperties(body.GetMass(), body.GetInertia())
    body.SetPos(pos)
    body.SetRot(chrono.Q_from_EulerAngles(
        random.uniform(0, math.pi),
        random.uniform(0, math.pi),
        random.uniform(0, math.pi)
    ))
    
    body.SetVelocity(vel)
    system.Add(body)
    return body


for _ in range(num_particles):
    particle = generate_particle()
    particles.append(particle)


class CustomGravityForce:
    def __init__(self, particles, G=1.0):
        self.particles = particles
        self.G = G  

    def apply(self):
        for i, p1 in enumerate(self.particles):
            for j, p2 in enumerate(self.particles):
                if i >= j:
                    continue  
                
                dir = p2.GetPos() - p1.GetPos()
                dist_sq = dir.Length2()
                if dist_sq < 1e-4:
                    continue  
                dist = math.sqrt(dist_sq)
                
                mass1 = p1.GetMass()
                mass2 = p2.GetMass()
                force_mag = self.G * mass1 * mass2 / dist_sq
                
                force_vec = dir * (force_mag / dist)
                
                p1.ApplyForce(force_vec)
                p2.ApplyForce(-force_vec)


gravity_force = CustomGravityForce(particles, G=0.5)


visualization = animate.ChApiIrrlicht()
visualization.SetSystem(system)
visualization.Initialize()
visualization.AddTypicalSky()
visualization.AddTypicalCamera(chrono.ChVectorD(0, 2, 5))
visualization.AddTypicalLights()


time_step = 0.01
total_time = 10.0
current_time = 0.0


while visualization.GetDevice().run():
    
    gravity_force.apply()

    
    system.DoStepDynamics(time_step)

    

    
    visualization.BeginScene()
    visualization.DrawAll()
    visualization.EndScene()

    
    current_time += time_step
    if current_time >= total_time:
        break


visualization.GetDevice().closeDevice()