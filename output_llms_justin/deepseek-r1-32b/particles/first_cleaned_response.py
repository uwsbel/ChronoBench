importchrono
import math
import random
from chrono import ChVector, ChQuaternion, ChSystem, ChIrrApp, ChIrrGui, ChCollisionModel, ChBody, ChSphere, ChBox, ChCylinder


system = ChSystem()
system.Set_G_acc(ChVector(0, 0, 0))  


application = ChIrrApp(system, "Particle Attraction Simulation", ChVector(1024, 768, 0))
application.AddTypicalLights()
application.AddTypicalLogo()
application.SetCameraPosition(ChVector(0, 50, 50))
application.SetCameraRotation(ChQuaternion(0.7071, 0, 0.7071, 0))


num_particles = 50
particle_mass = 1.0
gravitational_constant = 0.001  

def create_particle(shape_type):
    
    particle = ChBody()
    system.AddBody(particle)
    particle.SetMass(particle_mass)
    
    
    x = random.uniform(-5, 5)
    y = random.uniform(-5, 5)
    z = random.uniform(-5, 5)
    particle.SetPos(ChVector(x, y, z))
    
    
    vx = random.uniform(-1, 1)
    vy = random.uniform(-1, 1)
    vz = random.uniform(-1, 1)
    particle.SetPos_dt(ChVector(vx, vy, vz))
    
    
    angle = random.uniform(0, 2 * math.pi)
    axis = ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    particle.SetRot(ChQuaternion(angle, axis))
    
    
    if shape_type == 'sphere':
        sphere = ChSphere()
        sphere.SetRadius(random.uniform(0.1, 0.5))
        particle.AddGeometry(sphere)
    elif shape_type == 'box':
        box = ChBox()
        box.SetDimensions(ChVector(random.uniform(0.2, 1), random.uniform(0.2, 1), random.uniform(0.2, 1)))
        particle.AddGeometry(box)
    elif shape_type == 'cylinder':
        cylinder = ChCylinder()
        cylinder.SetHeight(random.uniform(0.5, 2))
        cylinder.SetRadius(random.uniform(0.1, 0.5))
        particle.AddGeometry(cylinder)
    
    particle.GetGeometry().SetMaterialColor(random.uniform(0,1), random.uniform(0,1), random.uniform(0,1))
    particle.SetCollide(False)  
    
    return particle


particles = []
for _ in range(num_particles):
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    particles.append(create_particle(shape_type))

def apply_gravitational_forces():
    
    for i in range(len(particles)):
        for j in range(i+1, len(particles)):
            
            pos_i = particles[i].GetPos()
            pos_j = particles[j].GetPos()
            delta = pos_j - pos_i
            distance = delta.Length()
            
            if distance > 0:
                
                force_magnitude = gravitational_constant * (particle_mass * particle_mass) / (distance * distance)
                force_direction = delta.Normalize()
                
                
                particles[i].AddForce(force_direction * force_magnitude)
                particles[j].AddForce(-force_direction * force_magnitude)


application.Begin()
while application.GetDevice().Run():
    
    for particle in particles:
        particle.ClearForces()
    
    
    apply_gravitational_forces()
    
    
    for particle in particles:
        velocity = particle.GetPos_dt()
        damping_force = -0.1 * velocity
        particle.AddForce(damping_force)
    
    
    system.DoStepDynamics(0.005)
    
    
    application.GetDevice().Render()
    
    
    if system.GetChTime() > 10:
        break

application.End()