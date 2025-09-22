import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import random
import math


print("Creating the simulation system...")
chrono_system = chrono.ChSystemNSC()
chrono_system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


chrono_system.SetSolverType(chrono.ChSolver.Type_PSOR)
chrono_system.SetSolverMaxIterations(100)
chrono_system.SetSolverForceTolerance(1e-8)


GRAVITATIONAL_CONSTANT = 6.67430e-2  
EMISSION_FREQUENCY = 0.5  
MAX_PARTICLES = 100  
SIMULATION_BOUNDS = 20  
PARTICLE_DENSITY = 1000.0  
MIN_PARTICLE_MASS = 0.5
MAX_PARTICLE_MASS = 10.0
MIN_PARTICLE_SIZE = 0.2
MAX_PARTICLE_SIZE = 0.8


particles = []
last_emission_time = 0


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(chrono_system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Particle Gravitational Attraction Simulation")
vis.Initialize()
vis.AddCamera(chrono.ChVectorD(0, 10, -20), chrono.ChVectorD(0, 0, 0))
vis.AddTypicalLights()


def create_random_particle():
    
    shape_type = random.randint(0, 2)
    
    
    pos_x = random.uniform(-SIMULATION_BOUNDS/2, SIMULATION_BOUNDS/2)
    pos_y = random.uniform(-SIMULATION_BOUNDS/2, SIMULATION_BOUNDS/2)
    pos_z = random.uniform(-SIMULATION_BOUNDS/2, SIMULATION_BOUNDS/2)
    
    
    vel_x = random.uniform(-1.0, 1.0)
    vel_y = random.uniform(-1.0, 1.0)
    vel_z = random.uniform(-1.0, 1.0)
    
    
    rot_angle = random.uniform(0, 2*math.pi)
    rot_axis_x = random.uniform(-1.0, 1.0)
    rot_axis_y = random.uniform(-1.0, 1.0)
    rot_axis_z = random.uniform(-1.0, 1.0)
    rot_quat = chrono.Q_from_AngAxis(rot_angle, chrono.ChVectorD(rot_axis_x, rot_axis_y, rot_axis_z))
    
    
    body = chrono.ChBody()
    body.SetPos(chrono.ChVectorD(pos_x, pos_y, pos_z))
    body.SetRot(rot_quat)
    body.SetPos_dt(chrono.ChVectorD(vel_x, vel_y, vel_z))
    
    
    material = chrono.ChMaterialSurfaceNSC()
    material.SetFriction(0.2)
    material.SetRestitution(0.8)
    
    
    size = random.uniform(MIN_PARTICLE_SIZE, MAX_PARTICLE_SIZE)
    
    
    if shape_type == 0:  
        body.AddVisualShape(chrono.ChVisualShapeSphere(size))
        body.AddCollisionShape(chrono.ChCollisionShapeSphere(size), chrono.ChVectorD(0, 0, 0))
        volume = (4/3) * math.pi * size**3
    elif shape_type == 1:  
        body.AddVisualShape(chrono.ChVisualShapeBox(size, size, size))
        body.AddCollisionShape(chrono.ChCollisionShapeBox(size, size, size), chrono.ChVectorD(0, 0, 0))
        volume = size**3
    else:  
        radius = size
        height = size * 2
        body.AddVisualShape(chrono.ChVisualShapeCylinder(radius, height))
        body.AddCollisionShape(chrono.ChCollisionShapeCylinder(radius, height), chrono.ChVectorD(0, 0, 0))
        volume = math.pi * radius**2 * height
    
    
    mass = random.uniform(MIN_PARTICLE_MASS, MAX_PARTICLE_MASS)
    body.SetMass(mass)
    
    
    inertia = chrono.ChVectorD(1, 1, 1)  
    if shape_type == 0:  
        inertia.x = inertia.y = inertia.z = (2/5) * mass * size**2
    elif shape_type == 1:  
        inertia.x = inertia.y = inertia.z = (1/6) * mass * size**2
    else:  
        inertia.x = inertia.y = (1/12) * mass * (3*radius**2 + height**2)
        inertia.z = (1/2) * mass * radius**2
    
    body.SetInertiaXX(inertia)
    
    
    col_r = random.uniform(0.0, 1.0)
    col_g = random.uniform(0.0, 1.0)
    col_b = random.uniform(0.0, 1.0)
    
    
    for vshape in body.GetVisualShapeList():
        vshape.SetColor(chrono.ChColor(col_r, col_g, col_b))
    
    
    chrono_system.Add(body)
    
    return body


class GravitationalForceCallback(chrono.PyChForceCallbackNSC):
    def __init__(self):
        super().__init__()
    
    def UpdateForce(self, time, body, force, torque):
        
        if body not in particles:
            return
        
        
        for other_body in particles:
            if other_body == body:
                continue
                
            
            pos1 = body.GetPos()
            pos2 = other_body.GetPos()
            r_vec = chrono.ChVectorD(pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z)
            
            
            dist = max(r_vec.Length(), 0.1)
            
            
            force_mag = GRAVITATIONAL_CONSTANT * body.GetMass() * other_body.GetMass() / (dist**2)
            
            
            r_hat = r_vec / dist
            
            
            grav_force = chrono.ChVectorD(
                force_mag * r_hat.x,
                force_mag * r_hat.y,
                force_mag * r_hat.z
            )
            
            
            force.x += grav_force.x
            force.y += grav_force.y
            force.z += grav_force.z


grav_force_callback = GravitationalForceCallback()


print("Starting the simulation...")
is_running = True

while is_running and vis.Run():
    
    vis.BeginScene()
    vis.Render()
    
    
    time = chrono_system.GetChTime()
    
    
    if time - last_emission_time > EMISSION_FREQUENCY and len(particles) < MAX_PARTICLES:
        new_particle = create_random_particle()
        particles.append(new_particle)
        new_particle.RegisterForceFunctor(grav_force_callback)
        last_emission_time = time
    
    
    particles_to_remove = []
    for particle in particles:
        pos = particle.GetPos()
        dist = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
        if dist > SIMULATION_BOUNDS * 2:
            particles_to_remove.append(particle)
    
    for particle in particles_to_remove:
        chrono_system.Remove(particle)
        particles.remove(particle)
    
    
    vis.DrawTextBackground(f"Particles: {len(particles)}/{MAX_PARTICLES}", 10, 30)
    vis.DrawTextBackground(f"Simulation time: {time:.2f} s", 10, 60)
    
    
    chrono_system.DoStepDynamics(0.01)
    
    
    vis.EndScene()

print("Simulation ended.")