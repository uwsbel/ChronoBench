import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


G = 0.1        
MAX_PARTICLES = 50
EMIT_INTERVAL = 0.1  
EMIT_RADIUS = 2.0    


class CustomGravity(chrono.Force):
    def __init__(self, particle_list):
        super().__init__()
        self.particles = particle_list
        self.G = G

    def Update(self, time, update_assets=True):
        n = len(self.particles)
        for i in range(n):
            body_i = self.particles[i]
            for j in range(i + 1, n):
                body_j = self.particles[j]
                
                
                r_vec = body_j.GetPos() - body_i.GetPos()
                r = r_vec.Length()
                
                
                if r < 1e-3:
                    continue
                
                
                force_mag = self.G * body_i.GetMass() * body_j.GetMass() / (r * r)
                force_dir = r_vec / r
                force = force_dir * force_mag
                
                
                body_i.AddForce(force)
                body_j.AddForce(-force)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


particles = []
gravity_force = CustomGravity(particles)
system.Add(gravity_force)


def create_particle():
    
    shape_type = random.choice(['sphere', 'box', 'cylinder'])
    density = 1000  
    
    
    if shape_type == 'sphere':
        radius = random.uniform(0.1, 0.3)
        body = chrono.ChBodyEasySphere(radius, density, True, False)
    elif shape_type == 'box':
        size = chrono.ChVectorD(*[random.uniform(0.1, 0.3) for _ in range(3)])
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, density, True, False)
    else:  
        radius = random.uniform(0.1, 0.2)
        height = random.uniform(0.2, 0.4)
        body = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, height, density, True, False)
    
    
    angle = random.uniform(0, 2 * math.pi)
    distance = random.uniform(0, EMIT_RADIUS)
    pos = chrono.ChVectorD(
        distance * math.cos(angle),
        random.uniform(3, 5),
        distance * math.sin(angle)
    )
    
    
    vel = chrono.ChVectorD(
        random.uniform(-0.5, 0.5),
        random.uniform(-1, 0),
        random.uniform(-0.5, 0.5)
    )
    ang_vel = chrono.ChVectorD(
        random.uniform(-2, 2),
        random.uniform(-2, 2),
        random.uniform(-2, 2)
    )
    
    
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetWvel_par(ang_vel)
    body.GetVisualShape(0).SetColor(chrono.ChColor(random.random(), random.random(), random.random()))
    
    
    system.Add(body)
    particles.append(body)
    return body


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 5, -10))
vis.AddTypicalLights()


particle_text = vis.GetGUIEnvironment().addStaticText(
    "Particles: 0", 
    chronoirr.recti(10, 10, 400, 40), 
    False
)


time = 0.0
next_emit_time = 0.0
sim_time = 0


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    particle_text.setText(f"Particles: {len(particles)} Time: {sim_time:.1f}s")
    
    
    if time >= next_emit_time and len(particles) < MAX_PARTICLES:
        create_particle()
        next_emit_time = time + EMIT_INTERVAL
    
    
    vis.EndScene()
    system.DoStepDynamics(1e-3)
    time += 1e-3
    sim_time += 1