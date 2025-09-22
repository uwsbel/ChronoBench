import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random
import math


G = 0.1  
MAX_PARTICLES = 50
EMIT_INTERVAL = 0.2  
EMIT_BOX_SIZE = chrono.ChVectorD(5, 1, 5)  


class GravitationalForce(chrono.ChAction):
    def __init__(self, particle_list):
        super().__init__()
        self.particle_list = particle_list
        self.G = G

    def ActionPerformed(self):
        n = len(self.particle_list)
        for i in range(n):
            body_i = self.particle_list[i]
            pos_i = body_i.GetPos()
            mass_i = body_i.GetMass()
            for j in range(i + 1, n):
                body_j = self.particle_list[j]
                pos_j = body_j.GetPos()
                mass_j = body_j.GetMass()
                
                
                r_vect = pos_j - pos_i
                r_len = r_vect.Length()
                
                if r_len > 0.01:  
                    
                    force_mag = self.G * mass_i * mass_j / (r_len * r_len)
                    force_dir = r_vect / r_len
                    force = force_mag * force_dir
                    
                    
                    body_i.Accumulate_force(force, pos_i, False)
                    body_j.Accumulate_force(-force, pos_j, False)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  
particles = []  


grav_force = GravitationalForce(particles)
system.Add(grav_force)


application = chronoirr.ChIrrApp(system, "Particle Gravity", chronoirr.dimension2du(800, 600))
application.AddLogo()
application.AddSkyBox()
application.AddCamera(chrono.ChVectorD(0, 14, -20), chrono.ChVectorD(0, 0, 0))
application.AddLight(chrono.ChVectorD(30, 30, -30), 100)


def create_particle():
    
    pos = chrono.ChVectorD(
        random.uniform(-EMIT_BOX_SIZE.x/2, EMIT_BOX_SIZE.x/2),
        random.uniform(0, EMIT_BOX_SIZE.y),
        random.uniform(-EMIT_BOX_SIZE.z/2, EMIT_BOX_SIZE.z/2)
    )
    
    
    vel = chrono.ChVectorD(
        random.uniform(-0.5, 0.5),
        random.uniform(-0.5, 0.5),
        random.uniform(-0.5, 0.5)
    )
    
    
    quat = chrono.ChQuaternionD()
    quat.Q_from_AngAxis(random.uniform(0, 2 * math.pi), chrono.ChVectorD(0, 1, 0))
    
    
    if random.choice([True, False]):
        
        radius = random.uniform(0.3, 0.6)
        density = random.uniform(500, 1500)
        body = chrono.ChBodyEasySphere(radius, density)
    else:
        
        size = chrono.ChVectorD(
            random.uniform(0.3, 0.6),
            random.uniform(0.3, 0.6),
            random.uniform(0.3, 0.6)
        )
        density = random.uniform(500, 1500)
        body = chrono.ChBodyEasyBox(size.x, size.y, size.z, density)
    
    
    body.SetPos(pos)
    body.SetPos_dt(vel)
    body.SetRot(quat)
    body.SetCollide(True)
    
    
    body.GetVisualShape(0).SetColor(chrono.ChColor(
        random.uniform(0.3, 1.0),
        random.uniform(0.3, 1.0),
        random.uniform(0.3, 1.0)
    ))
    
    system.Add(body)
    particles.append(body)


application.AssetBindAll()
application.AssetUpdateAll()
time_step = 0.002
next_emit_time = 0


while application.GetDevice().run():
    current_time = system.GetChTime()
    
    
    if current_time >= next_emit_time and len(particles) < MAX_PARTICLES:
        create_particle()
        next_emit_time = current_time + EMIT_INTERVAL
        application.AssetBindAll()
        application.AssetUpdateAll()
    
    
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStepDynamics(time_step)
    application.EndScene()