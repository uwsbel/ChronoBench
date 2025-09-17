import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import random

# Initialize the Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  # Disable default gravity
system.SetTimestepperType(chrono.ChTimestepperType.DISCRETE)
system.GetSettings().timestep = 0.001

# Initialize Irrlicht visualizer
visualizer = chronoirr.ChVisualSystemIrrlicht()
visualizer.SetWindowSize(1024, 768)
visualizer.SetWindowTitle('Gravitational Particle Attraction')
visualizer.SetSymbolscale(0.01)
visualizer.SetCameraPosition(chrono.ChVectorD(5, 5, 5))
visualizer.SetCameraLookat(chrono.ChVectorD(0, 0, 0))
system.SetVisualSystem(visualizer)

particles = []
emit_interval = 0.5  # seconds between emissions
next_emit_time = 0.0

def emit_particle():
    pos = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    vel = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    )
    mass = random.uniform(1, 10)
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    
    body = chrono.ChBody()
    system.Add(body)
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetMass(mass)
    body.SetMaterialSurface(chrono.ChMaterialSurface())
    
    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.5)
        body.SetInertia(chrono.ChSphereInertia(mass, radius))
        body.GetCollisionModel().AddSphere(radius)
    elif shape_type == 'box':
        sx, sy, sz = random.uniform(0.2, 1), random.uniform(0.2, 1), random.uniform(0.2, 1)
        body.SetInertia(chrono.ChBoxInertia(mass, sx, sy, sz))
        body.GetCollisionModel().AddBox(sx/2, sy/2, sz/2)
    elif shape_type == 'cylinder':
        radius = random.uniform(0.1, 0.5)
        height = random.uniform(0.2, 1)
        body.SetInertia(chrono.ChCylinderInertia(mass, radius, height))
        body.GetCollisionModel().AddCylinder(radius, radius, height)
    
    body.GetCollisionModel().BuildModel()
    body.SetCollide(True)
    
    force = chrono.ChForce()
    body.AddForce(force)
    body.force = force  # Store force reference
    
    visualizer.AddBody(body)
    particles.append(body)

# Main simulation loop
while visualizer.Run():
    current_time = system.GetChTime()
    
    # Emit particles periodically
    if current_time >= next_emit_time:
        emit_particle()
        next_emit_time += emit_interval
    
    # Compute gravitational forces between all particles
    for particle in particles:
        total_force = chrono.ChVectorD(0, 0, 0)
        pos_a = particle.GetPos()
        mass_a = particle.GetMass()
        
        for other in particles:
            if other == particle:
                continue
            pos_b = other.GetPos()
            r_vec = pos_b - pos_a
            distance_sq = r_vec.Length2()
            
            if distance_sq < 1e-6:
                continue  # Avoid division by zero
            
            distance = np.sqrt(distance_sq)
            G = 1e-3  # Gravitational constant (adjust as needed)
            force_magnitude = G * mass_a * other.GetMass() / distance_sq
            unit_vec = r_vec / distance
            total_force += force_magnitude * unit_vec
        
        # Apply computed force to the particle
        particle.force.SetF(total_force)
    
    # Step the simulation and render
    system.DoStepDynamics()
    visualizer.DrawAll()