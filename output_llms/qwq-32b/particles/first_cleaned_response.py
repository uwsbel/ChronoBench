import pychrono as chrono
import random
import math
import numpy as np

def create_particle(system, particles_list, material):
    
    pos = chrono.ChVectorD(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(0, 5)
    )
    
    
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    
    
    mass = random.uniform(0.1, 10)
    
    
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetMass(mass)
    body.SetMaterialSurface(material)
    body.SetCollide(True)  
    
    
    body.GetCollisionModel().ClearModel()
    body.GetCollisionModel().AddSphere(0.1)
    body.GetCollisionModel().BuildModel()
    
    
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    
    if shape_type == 'sphere':
        vis = chrono.ChSphereShape()
        vis.GetSphereGeometry().rad = 0.1
    elif shape_type == 'box':
        hx = random.uniform(0.1, 0.3)
        hy = random.uniform(0.1, 0.3)
        hz = random.uniform(0.1, 0.3)
        vis = chrono.ChBoxShape(chrono.ChVectorD(hx, hy, hz))
    elif shape_type == 'cylinder':
        radius = random.uniform(0.1, 0.3)
        height = random.uniform(0.2, 0.6)
        vis = chrono.ChCylinderShape()
        vis.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -height/2)
        vis.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, height/2)
        vis.GetCylinderGeometry().rad = radius
    
    body.AddAsset(vis)
    
    
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(0, 1, 0)
    quat = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)
    
    
    system.Add(body)
    particles_list.append(body)

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    vis = chrono.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Gravitational Particle Attraction Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetCamPosition(chrono.ChVectorD(0, 0, 10))
    vis.SetCamRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    my_system.SetVisualSystem(vis)
    
    
    material = chrono.ChMaterialSurface()
    material.SetFriction(0.3)
    material.SetRestitution(0.5)
    
    
    particles = []
    
    
    num_particles = 50
    for _ in range(num_particles):
        create_particle(my_system, particles, material)
    
    
    vis.Begin()
    while vis.Run():
        
        for p in particles:
            p.ClearForces()
        
        
        for p in particles:
            net_force = chrono.ChVectorD(0, 0, 0)
            for o in particles:
                if p == o:
                    continue
                pos_p = p.GetPos()
                pos_o = o.GetPos()
                delta = pos_o - pos_p
                distance_sq = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z
                if distance_sq < 1e-6:
                    continue
                distance = math.sqrt(distance_sq)
                G = 1e-3  
                m_p = p.GetMass()
                m_o = o.GetMass()
                F = G * m_p * m_o / distance_sq
                direction = delta / distance
                force = F * direction
                net_force += force
            p.AddForce(net_force)
        
        
        my_system.DoStepDynamics(0.005)  
        
        
        vis.Render()
    
    vis.End()

if __name__ == '__main__':
    main()

import pychrono as chrono
import random
import math
import numpy as np

def create_particle(system, particles_list, material):
    
    pos = chrono.ChVectorD(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(0, 5)
    )
    
    
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    
    
    mass = random.uniform(0.1, 10)
    
    
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetMass(mass)
    body.SetMaterialSurface(material)
    body.SetCollide(True)  
    
    
    body.GetCollisionModel().ClearModel()
    body.GetCollisionModel().AddSphere(0.1)
    body.GetCollisionModel().BuildModel()
    
    
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    
    if shape_type == 'sphere':
        vis = chrono.ChSphereShape()
        vis.GetSphereGeometry().rad = 0.1
    elif shape_type == 'box':
        hx = random.uniform(0.1, 0.3)
        hy = random.uniform(0.1, 0.3)
        hz = random.uniform(0.1, 0.3)
        vis = chrono.ChBoxShape(chrono.ChVectorD(hx, hy, hz))
    elif shape_type == 'cylinder':
        radius = random.uniform(0.1, 0.3)
        height = random.uniform(0.2, 0.6)
        vis = chrono.ChCylinderShape()
        vis.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -height/2)
        vis.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, height/2)
        vis.GetCylinderGeometry().rad = radius
    
    body.AddAsset(vis)
    
    
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(0, 1, 0)
    quat = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)
    
    
    system.Add(body)
    particles_list.append(body)

def main():
    
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
    
    
    vis = chrono.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Gravitational Particle Attraction Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetCamPosition(chrono.ChVectorD(0, 0, 10))
    vis.SetCamRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    my_system.SetVisualSystem(vis)
    
    
    material = chrono.ChMaterialSurface()
    material.SetFriction(0.3)
    material.SetRestitution(0.5)
    
    
    particles = []
    
    
    num_particles = 50
    for _ in range(num_particles):
        create_particle(my_system, particles, material)
    
    
    vis.Begin()
    while vis.Run():
        
        for p in particles:
            p.ClearForces()
        
        
        for p in particles:
            net_force = chrono.ChVectorD(0, 0, 0)
            for o in particles:
                if p == o:
                    continue
                pos_p = p.GetPos()
                pos_o = o.GetPos()
                delta = pos_o - pos_p
                distance_sq = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z
                if distance_sq < 1e-6:
                    continue
                distance = math.sqrt(distance_sq)
                G = 1e-3  
                m_p = p.GetMass()
                m_o = o.GetMass()
                F = G * m_p * m_o / distance_sq
                direction = delta / distance
                force = F * direction
                net_force += force
            p.AddForce(net_force)
        
        
        my_system.DoStepDynamics(0.005)  
        
        
        vis.Render()
    
    vis.End()

if __name__ == '__main__':
    main()