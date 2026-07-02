import pychrono as chrono
import pychrono.irrlicht as irr
import numpy as np
import random


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


application = irr.ChIrrApp(system, 'Gravitational Particle Simulation', irr.dimension2du(800, 600))
application.AddLogo()
application.AddCamera(chrono.ChVectorD(0, 3, -10))
application.AddLightWithShadow(chrono.ChVectorD(5, 5, -5), chrono.ChVectorD(0, 0, 0), 5, 20, 5, 50, 1024)
application.SetShowInfos(True)
application.SetWindowSize(1024, 768)
application.SetRenderWireframe(False)
application.SetTryRealtime(True)


G = 0.001


particles = []


class GravitationalForce(chrono.ChCustomForce):
    def __init__(self, particles_list, G_const):
        chrono.ChCustomForce.__init__(self)
        self.particles = particles_list
        self.G = G_const

    def Update(self, udt):
        body = self.GetBody()
        if body not in self.particles:
            return
        Fx = 0
        Fy = 0
        Fz = 0
        for other in self.particles:
            if other == body:
                continue
            pos1 = body.GetPos()
            pos2 = other.GetPos()
            dx = pos2.x - pos1.x
            dy = pos2.y - pos1.y
            dz = pos2.z - pos1.z
            r2 = dx*dx + dy*dy + dz*dz
            r = np.sqrt(r2)
            if r < 0.1:  
                continue
            mass1 = body.GetMass()
            mass2 = other.GetMass()
            force_magnitude = self.G * mass1 * mass2 / r2
            Fx += force_magnitude * dx / r
            Fy += force_magnitude * dy / r
            Fz += force_magnitude * dz / r
        self.SetForce(Fx, Fy, Fz)

def add_particle():
    
    body = chrono.ChBody()
    
    pos_x = random.uniform(-5, 5)
    pos_y = random.uniform(0, 2)
    pos_z = random.uniform(-5, 5)
    body.SetPos(chrono.ChVectorD(pos_x, pos_y, pos_z))
    
    vel_x = random.uniform(-0.1, 0.1)
    vel_y = random.uniform(-0.1, 0.1)
    vel_z = random.uniform(-0.1, 0.1)
    body.SetPos_dt(chrono.ChVectorD(vel_x, vel_y, vel_z))
    
    angle = random.uniform(0, 2 * np.pi)
    axis = chrono.ChVectorD(
        random.gauss(0, 1),
        random.gauss(0, 1),
        random.gauss(0, 1)
    )
    axis.Normalize()
    q = chrono.Q_from_AngAxis(angle, axis)
    body.SetRot(q)
    
    if random.random() < 0.5:
        
        radius = random.uniform(0.1, 0.3)
        volume = (4.0/3.0) * np.pi * (radius ** 3)
        shape = chrono.ChVisualShapeSphere(radius)
    else:
        
        half_width = random.uniform(0.1, 0.3)
        half_height = random.uniform(0.1, 0.3)
        half_depth = random.uniform(0.1, 0.3)
        volume = 8 * half_width * half_height * half_depth  
        shape = chrono.ChVisualShapeBox(half_width, half_height, half_depth)
    body.SetMass(volume)
    body.AddVisualShape(shape)
    
    system.AddBody(body)
    particles.append(body)
    
    force = GravitationalForce(particles, G)
    body.AddForce(force)


time_since_last_emit = 0.0
emit_interval = 0.5  


application.SetTimestep(0.01)


while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.EndScene()
    
    
    system.DoStepDynamics(application.GetTimestep())
    
    
    time_since_last_emit += application.GetTimestep()
    if time_since_last_emit >= emit_interval:
        add_particle()
        time_since_last_emit = 0.0