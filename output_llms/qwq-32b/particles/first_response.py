import pychrono as chrono
import random
import math
import numpy as np

def create_particle(system, particles_list, material):
    # Random position within a box (-5,5) in x,y, (0,5) in z
    pos = chrono.ChVectorD(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(0, 5)
    )
    
    # Random velocity components
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    
    # Random mass between 0.1 and 10 kg
    mass = random.uniform(0.1, 10)
    
    # Create body
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetMass(mass)
    body.SetMaterialSurface(material)
    body.SetCollide(True)  # Enable collision
    
    # Collision shape: sphere with radius 0.1
    body.GetCollisionModel().ClearModel()
    body.GetCollisionModel().AddSphere(0.1)
    body.GetCollisionModel().BuildModel()
    
    # Visual shape: random between sphere, box, cylinder
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
    
    # Random orientation (rotation around Y-axis)
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(0, 1, 0)
    quat = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)
    
    # Add to system and list
    system.Add(body)
    particles_list.append(body)

def main():
    # Create the Chrono system
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity
    
    # Create Irrlicht visualizer
    vis = chrono.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Gravitational Particle Attraction Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetCamPosition(chrono.ChVectorD(0, 0, 10))
    vis.SetCamRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    my_system.SetVisualSystem(vis)
    
    # Material properties for collisions
    material = chrono.ChMaterialSurface()
    material.SetFriction(0.3)
    material.SetRestitution(0.5)
    
    # List to hold all particles
    particles = []
    
    # Emit particles
    num_particles = 50
    for _ in range(num_particles):
        create_particle(my_system, particles, material)
    
    # Simulation loop
    vis.Begin()
    while vis.Run():
        # Clear forces on all particles
        for p in particles:
            p.ClearForces()
        
        # Compute gravitational forces between all pairs
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
                G = 1e-3  # Gravitational constant (scaled for simulation)
                m_p = p.GetMass()
                m_o = o.GetMass()
                F = G * m_p * m_o / distance_sq
                direction = delta / distance
                force = F * direction
                net_force += force
            p.AddForce(net_force)
        
        # Advance the simulation
        my_system.DoStepDynamics(0.005)  # Time step of 5ms
        
        # Update visualization
        vis.Render()
    
    vis.End()

if __name__ == '__main__':
    main()

import pychrono as chrono
import random
import math
import numpy as np

def create_particle(system, particles_list, material):
    # Random position within a box (-5,5) in x,y, (0,5) in z
    pos = chrono.ChVectorD(
        random.uniform(-5, 5),
        random.uniform(-5, 5),
        random.uniform(0, 5)
    )
    
    # Random velocity components
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    
    # Random mass between 0.1 and 10 kg
    mass = random.uniform(0.1, 10)
    
    # Create body
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetMass(mass)
    body.SetMaterialSurface(material)
    body.SetCollide(True)  # Enable collision
    
    # Collision shape: sphere with radius 0.1
    body.GetCollisionModel().ClearModel()
    body.GetCollisionModel().AddSphere(0.1)
    body.GetCollisionModel().BuildModel()
    
    # Visual shape: random between sphere, box, cylinder
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
    
    # Random orientation (rotation around Y-axis)
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(0, 1, 0)
    quat = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)
    
    # Add to system and list
    system.Add(body)
    particles_list.append(body)

def main():
    # Create the Chrono system
    my_system = chrono.ChSystemNSC()
    my_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity
    
    # Create Irrlicht visualizer
    vis = chrono.ChVisualSystemIrrlicht()
    vis.SetWindowSize(1024, 768)
    vis.SetWindowTitle('Gravitational Particle Attraction Simulation')
    vis.SetSymbolscale(0.01)
    vis.SetCamPosition(chrono.ChVectorD(0, 0, 10))
    vis.SetCamRotation(chrono.ChQuaternionD(1, 0, 0, 0))
    my_system.SetVisualSystem(vis)
    
    # Material properties for collisions
    material = chrono.ChMaterialSurface()
    material.SetFriction(0.3)
    material.SetRestitution(0.5)
    
    # List to hold all particles
    particles = []
    
    # Emit particles
    num_particles = 50
    for _ in range(num_particles):
        create_particle(my_system, particles, material)
    
    # Simulation loop
    vis.Begin()
    while vis.Run():
        # Clear forces on all particles
        for p in particles:
            p.ClearForces()
        
        # Compute gravitational forces between all pairs
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
                G = 1e-3  # Gravitational constant (scaled for simulation)
                m_p = p.GetMass()
                m_o = o.GetMass()
                F = G * m_p * m_o / distance_sq
                direction = delta / distance
                force = F * direction
                net_force += force
            p.AddForce(net_force)
        
        # Advance the simulation
        my_system.DoStepDynamics(0.005)  # Time step of 5ms
        
        # Update visualization
        vis.Render()
    
    vis.End()

if __name__ == '__main__':
    main()