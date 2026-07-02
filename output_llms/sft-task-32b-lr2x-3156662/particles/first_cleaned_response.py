import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, 0))  


particles = []

def add_particle():
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    radius = random.uniform(0.05, 0.15)
    size = chrono.ChVector3d(random.uniform(0.1, 0.3), random.uniform(0.1, 0.3), random.uniform(0.1, 0.3))
    density = 1000.0
    pos = chrono.ChVector3d(random.uniform(-2, 2), random.uniform(-2, 2), random.uniform(0, 5))
    color = chrono.ChColor(random.random(), random.random(), random.random())
    
    if shape_type == 'sphere':
        body = chrono.ChBodyEasySphere(radius, density, True, True)
    elif shape_type == 'box':
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, density, True, True)
    elif shape_type == 'cylinder':
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis.Z, radius, max(size.y, 0.1), density, True, True)
    
    
    vel = chrono.ChVector3d(random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1))
    body.SetPos(pos)
    body.SetPos_dt(vel)
    q = chrono.ChQuaterniond()
    q.FromAngleAxis(chrono.CH_C_PI * random.random(), 
                     chrono.ChVector3d(random.random(), random.random(), random.random()).GetNormalized())
    body.SetRot(q)
    body.SetColor(color)
    system.Add(body)
    particles.append(body)


for _ in range(50):
    add_particle()

def compute_gravitational_forces():
    G = 6.67430e-1  
    for i, body_i in enumerate(particles):
        pos_i = body_i.GetPos()
        mass_i = body_i.GetMass()
        total_force = chrono.ChVector3d(0, 0, 0)
        for j, body_j in enumerate(particles):
            if i == j:
                continue
            pos_j = body_j.GetPos()
            vec = pos_i - pos_j
            distance_sq = vec.Length2()
            if distance_sq < 1e-9:
                continue  
            distance = math.sqrt(distance_sq)
            mass_j = body_j.GetMass()
            force_magnitude = G * mass_i * mass_j / distance_sq
            direction = vec / distance
            total_force += direction * force_magnitude
        body_i.ClearForces()
        body_i.ApplyForce(total_force, pos_i)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('PyChrono Gravitational Particles')
vis.Initialize()
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, -5, 5), chrono.ChVector3d(0, 0, 0))
vis.AddTypicalLights()


step = 0.001
next_emit_time = 0.0

while vis.Run():
    current_time = system.GetChTime()
    
    if current_time >= next_emit_time:
        add_particle()
        next_emit_time += 0.1
    
    compute_gravitational_forces()
    
    system.DoStepDynamics(step)
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()