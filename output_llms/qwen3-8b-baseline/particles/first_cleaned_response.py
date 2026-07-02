import pychrono as chrono
from pychrono import irrlicht
import numpy as np
import random


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVectorD(0, 0, 0))  


application = irrlicht.ChIrrApp(system, 'Particle Gravity Simulation', irrlicht.dimension2d_i(800, 600))
application.AddCamera(chrono.ChVectorD(0, 2, -5))
application.SetTimestep(0.01)


particles = []

def emit_particle():
    
    shape_type = random.choice(['sphere', 'box'])
    
    mass = random.uniform(1, 5)
    
    position = chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5))
    
    velocity = chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    
    angle = random.uniform(0, 2 * np.pi)
    axis = np.random.rand(3)
    axis /= np.linalg.norm(axis)
    quat = chrono.ChQuaternionD(np.cos(angle/2), axis[0]*np.sin(angle/2), axis[1]*np.sin(angle/2), axis[2]*np.sin(angle/2))
    
    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.5)
        body = chrono.ChBodyEasySphere(radius, mass, position, True, True)
        body.SetRot(quat)
        body.SetVelocity(velocity)
    else:
        length = random.uniform(0.2, 0.6)
        width = random.uniform(0.2, 0.6)
        height = random.uniform(0.2, 0.6)
        body = chrono.ChBodyEasyBox(length, width, height, mass, position, True, True)
        body.SetRot(quat)
        body.SetVelocity(velocity)
    
    system.Add(body)
    particles.append(body)


emit_particle()

step_count = 0
while application.GetApplication().IsRunning():
    application.BeginScene()
    application.DrawAll()
    
    
    if step_count % 100 == 0:
        emit_particle()
    
    
    for i in range(len(particles)):
        for j in range(i + 1, len(particles)):
            body_i = particles[i]
            body_j = particles[j]
            pos_i = body_i.GetPos()
            pos_j = body_j.GetPos()
            r_ij = pos_j - pos_i
            r_ij_length = r_ij.Length()
            if r_ij_length < 1e-3:
                continue
            G = 1.0  
            mass_i = body_i.GetMass()
            mass_j = body_j.GetMass()
            force_magnitude = G * mass_i * mass_j / (r_ij_length ** 2)
            force_vector = r_ij / r_ij_length * force_magnitude
            force_on_i = force_vector
            force_on_j = -force_vector
            body_i.AddForce(force_on_i)
            body_j.AddForce(force_on_j)
    
    application.DoStep()
    application.EndScene()
    step_count += 1

application.Close()