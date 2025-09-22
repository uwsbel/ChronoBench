import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


G = 6.67430e-2  
EMISSION_INTERVAL = 0.2
MAX_PARTICLES = 20
SIMULATION_TIME = 10


class GravitationalForce(chrono.ChForce):
    def __init__(self, body, particle_list):
        super().__init__()
        self.body = body
        self.particle_list = particle_list
        self.SetMode(chrono.ChForce.FORCE)
    
    def update(self):
        total_force = chrono.ChVectorD(0, 0, 0)
        for other in self.particle_list:
            if other == self.body:
                continue
            r_vec = other.GetPos() - self.body.GetPos()
            distance = r_vec.Length()
            if distance < 1e-5:
                continue
            force_mag = G * self.body.GetMass() * other.GetMass() / (distance**2)
            force_dir = r_vec / distance
            total_force += force_mag * force_dir
        self.SetForce(total_force)


system = chrono.ChSystemSMC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


material = chrono.ChMaterialSurfaceSMC()
material.SetFriction(0.4)
material.SetRestitution(0.2)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gravitational Particle Attraction')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 3, -4))
vis.AddTypicalLights()


particles = []
gravity_forces = []
next_emission_time = 0


def create_particle():
    
    pos = chrono.ChVectorD(
        random.uniform(-0.5, 0.5),
        random.uniform(-0.2, 0.2),
        random.uniform(-0.5, 0.5)
    )
    
    
    vel = chrono.ChVectorD(
        random.uniform(-0.3, 0.3),
        random.uniform(-0.1, 0.1),
        random.uniform(-0.3, 0.3)
    )
    
    
    angle = random.uniform(0, 2 * math.pi)
    axis = chrono.ChVectorD(
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        random.uniform(-1, 1)
    ).GetNormalized()
    rot = chrono.Q_from_AngAxis(angle, axis)
    
    
    body = chrono.ChBody()
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetRot(rot)
    body.SetCollide(True)
    
    
    shape_type = random.choice(['sphere', 'box'])
    density = random.uniform(500, 1500)  
    
    if shape_type == 'sphere':
        radius = random.uniform(0.05, 0.15)
        mass = density * (4/3) * math.pi * radius**3
        body.SetMass(mass)
        body.SetInertiaXX(chrono.ChVectorD(
            0.4 * mass * radius**2,
            0.4 * mass * radius**2,
            0.4 * mass * radius**2
        ))
        
        
        body.GetCollisionModel().ClearModel()
        body.GetCollisionModel().AddSphere(material, radius)
        body.GetCollisionModel().BuildModel()
        
        vis_shape = chrono.ChSphereShape(radius)
        vis_shape.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        body.AddVisualShape(vis_shape)
    
    else:  
        size = chrono.ChVectorD(
            random.uniform(0.05, 0.15),
            random.uniform(0.05, 0.15),
            random.uniform(0.05, 0.15)
        )
        mass = density * size.x * size.y * size.z
        body.SetMass(mass)
        body.SetInertiaXX(chrono.ChVectorD(
            mass/12 * (size.y**2 + size.z**2),
            mass/12 * (size.x**2 + size.z**2),
            mass/12 * (size.x**2 + size.y**2)
        ))
        
        
        body.GetCollisionModel().ClearModel()
        body.GetCollisionModel().AddBox(material, size.x, size.y, size.z)
        body.GetCollisionModel().BuildModel()
        
        vis_shape = chrono.ChBoxShape(size)
        vis_shape.SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
        body.AddVisualShape(vis_shape)
    
    return body


time_step = 0.001
while system.GetChTime() < SIMULATION_TIME:
    current_time = system.GetChTime()
    
    
    if current_time >= next_emission_time and len(particles) < MAX_PARTICLES:
        new_particle = create_particle()
        system.Add(new_particle)
        particles.append(new_particle)
        
        
        gf = GravitationalForce(new_particle, particles)
        new_particle.AddForce(gf)
        gravity_forces.append(gf)
        
        next_emission_time += EMISSION_INTERVAL
    
    
    for gf in gravity_forces:
        gf.update()
    
    
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    system.DoStepDynamics(time_step)