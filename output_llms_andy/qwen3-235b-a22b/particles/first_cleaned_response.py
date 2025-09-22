import pychrono as chrono
import pychrono.irrlicht as irrlicht
import random
import math


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVector(0, 0, 0))  

def create_particle(system):
    
    body = chrono.ChBody()
    
    
    pos = chrono.ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    body.SetPos(pos)
    
    
    vel = chrono.ChVector(random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1), random.uniform(-0.1, 0.1))
    body.SetPos_dt(vel)
    
    
    axis = chrono.ChVector(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
    axis.Normalize()
    angle = random.uniform(0, 2 * math.pi)
    quat = chrono.ChQuaternion()
    quat.Q_from_AngAxis(angle, axis)
    body.SetRot(quat)
    
    
    shape = random.choice(['sphere', 'box', 'cylinder'])
    material = chrono.ChMaterialSurfaceNSC()
    mass = random.uniform(0.5, 2.0)  
    
    if shape == 'sphere':
        radius = random.uniform(0.1, 0.3)
        
        body.GetCollisionModel().AddSphere(material, radius, chrono.ChVector(0, 0, 0))
        
        inertia = (0.4 * mass * radius**2) * chrono.ChVector(1, 1, 1)
        
        sphere_shape = chrono.ChSphereShape()
        sphere_shape.GetSphereGeometry().rad = radius
        body.AddAsset(sphere_shape)
        
    elif shape == 'box':
        size = chrono.ChVector(random.uniform(0.2, 0.5), random.uniform(0.2, 0.5), random.uniform(0.2, 0.5))
        
        body.GetCollisionModel().AddBox(material, size.x, size.y, size.z, chrono.ChVector(0, 0, 0))
        
        ix = (mass / 6.0) * (size.y**2 + size.z**2)
        iy = (mass / 6.0) * (size.x**2 + size.z**2)
        iz = (mass / 6.0) * (size.x**2 + size.y**2)
        inertia = chrono.ChVector(ix, iy, iz)
        
        box_shape = chrono.ChBoxShape()
        box_shape.GetBoxGeometry().Size = size * 0.5  
        body.AddAsset(box_shape)
        
    elif shape == 'cylinder':
        radius = random.uniform(0.1, 0.3)
        height = random.uniform(0.2, 0.6)
        
        body.GetCollisionModel().AddCylinder(material, radius, height / 2, chrono.ChVector(0, 0, 0))
        
        iy = 0.5 * mass * radius**2
        ix = (mass / 12.0) * (3 * radius**2 + height**2)
        iz = ix
        inertia = chrono.ChVector(ix, iy, iz)
        
        cylinder_shape = chrono.ChCylinderShape()
        cylinder_shape.GetCylinderGeometry().rad = radius
        cylinder_shape.GetCylinderGeometry().p1 = chrono.ChVector(0, -height/2, 0)
        cylinder_shape.GetCylinderGeometry().p2 = chrono.ChVector(0, height/2, 0)
        body.AddAsset(cylinder_shape)
    
    
    body.SetMass(mass)
    body.SetInertiaXX(inertia)
    body.SetCollide(True)
    
    
    color = chrono.ChColorAsset()
    color.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    body.AddAsset(color)
    
    system.Add(body)
    return body


particles = [create_particle(system) for _ in range(20)]


application = irrlicht.ChIrrApp(system, 'Gravitational Particle Simulation', irrlicht.dimension2du(800, 600))
application.AddTypicalLights()
application.AddCamera(chrono.ChVector(0, 0, -5))
application.AssetBindAll()
application.AssetUpdateAll()


G = 1e-3  
while application.GetDevice().run():
    
    for body in particles:
        total_force = chrono.ChVector(0, 0, 0)
        for other in particles:
            if body == other:
                continue
            r = other.GetPos() - body.GetPos()
            distance_sq = r.x**2 + r.y**2 + r.z**2
            if distance_sq < 1e-8:  
                continue
            distance = math.sqrt(distance_sq)
            force_mag = G * body.GetMass() * other.GetMass() / distance_sq
            force = r * (force_mag / distance)  
            total_force += force
        body.Set_Force(total_force)
    
    
    system.DoStepDynamics(1e-3)
    
    
    application.BeginScene()
    application.DrawAll()
    application.EndScene()