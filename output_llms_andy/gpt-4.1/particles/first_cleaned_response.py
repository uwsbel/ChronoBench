import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


application = chronoirr.ChIrrApp(system, "Particle Gravity Simulation", chronoirr.dimension2du(1024,768))
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, -8))
application.AddTypicalLights()


particle_list = []

def random_shape():
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    if shape_type == 'sphere':
        radius = random.uniform(0.05, 0.15)
        body = chrono.ChBodyEasySphere(radius, 1000, True, True)
        body.SetMass(0.1)
    elif shape_type == 'box':
        size = chrono.ChVectorD(random.uniform(0.05, 0.15), random.uniform(0.05, 0.15), random.uniform(0.05, 0.15))
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True)
        body.SetMass(0.1)
    else:  
        radius = random.uniform(0.05, 0.12)
        height = random.uniform(0.05, 0.18)
        body = chrono.ChBodyEasyCylinder(radius, height, 1000, True, True)
        body.SetMass(0.1)
    return body

def emit_particle():
    body = random_shape()
    
    pos = chrono.ChVectorD(random.uniform(-1,1), random.uniform(0.5,2), random.uniform(-1,1))
    body.SetPos(pos)
    
    q = chrono.Q_from_AngAxis(random.uniform(0, math.pi), chrono.ChVectorD(random.random(), random.random(), random.random()))
    body.SetRot(q)
    
    vel = chrono.ChVectorD(random.uniform(-0.5,0.5), random.uniform(-0.5,0.5), random.uniform(-0.5,0.5))
    body.SetPos_dt(vel)
    
    body.SetBodyFixed(False)
    
    system.Add(body)
    particle_list.append(body)


def apply_particle_gravity():
    G = 0.2  
    n = len(particle_list)
    for i in range(n):
        for j in range(i+1, n):
            a = particle_list[i]
            b = particle_list[j]
            
            pa = a.GetPos()
            pb = b.GetPos()
            r_vec = pb - pa
            r = r_vec.Length()
            if r < 1e-4:
                continue  
            
            force_mag = G * a.GetMass() * b.GetMass() / (r*r)
            force_vec = force_mag * r_vec.GetNormalized()
            
            a.Empty_forces_accumulators()
            b.Empty_forces_accumulators()
            a.Accumulate_force(force_vec, pa, False)
            b.Accumulate_force(-force_vec, pb, False)


emit_interval = 0.3  
emit_timer = 0
max_particles = 20

application.AssetBindAll()
application.AssetUpdateAll()

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    
    
    if len(particle_list) < max_particles:
        emit_timer += application.GetStep()
        if emit_timer >= emit_interval:
            emit_particle()
            emit_timer = 0

    
    apply_particle_gravity()

    
    application.DoStep()
    application.EndScene()