import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
import random


class MutualGravity(chrono.Force):
    def __init__(self, G=0.1, min_distance=0.2):
        super().__init__()
        self.G = G  
        self.min_distance = min_distance  
    
    def ComputeForce(self, body, pos, vel, time):
        force = chrono.ChVectorD(0, 0, 0)
        
        if not hasattr(body, 'is_particle'):
            return force
        
        
        for other in self.system.GetBodies():
            if other == body or not hasattr(other, 'is_particle'):
                continue
                
            r_vec = other.GetPos() - body.GetPos()
            r = r_vec.Length()
            if r < self.min_distance:
                r = self.min_distance
                
            force_mag = self.G * body.GetMass() * other.GetMass() / (r * r)
            force += r_vec.GetNormalized() * force_mag
        
        return force


class ParticleEmitter:
    def __init__(self, system, gravity_force, 
                 pos_range=(-3, 3), vel_range=(-0.5, 0.5), 
                 mass_range=(0.1, 1.0), size_range=(0.1, 0.3)):
        self.system = system
        self.gravity_force = gravity_force
        self.pos_range = pos_range
        self.vel_range = vel_range
        self.mass_range = mass_range
        self.size_range = size_range
        self.last_emit_time = 0
    
    def emit(self, time, emit_interval=0.1):
        if time - self.last_emit_time < emit_interval:
            return
        
        self.last_emit_time = time
        
        
        particle = chrono.ChBody()
        particle.is_particle = True  
        
        
        mass = random.uniform(*self.mass_range)
        size = random.uniform(*self.size_range)
        
        
        pos = chrono.ChVectorD(
            random.uniform(*self.pos_range),
            random.uniform(*self.pos_range),
            random.uniform(*self.pos_range)
        )
        vel = chrono.ChVectorD(
            random.uniform(*self.vel_range),
            random.uniform(*self.vel_range),
            random.uniform(*self.vel_range)
        )
        
        particle.SetMass(mass)
        particle.SetPos(pos)
        particle.SetPos_dt(vel)
        
        
        if random.random() > 0.5:
            shape = chrono.ChSphereShape(size)
        else:
            shape = chrono.ChBoxShape(chrono.ChVectorD(size, size*0.7, size*1.2))
        
        particle.AddVisualShape(shape)
        particle.SetCollide(False)
        
        
        self.system.Add(particle)
        self.gravity_force.AddBody(particle)


system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


gravity = MutualGravity(G=0.1, min_distance=0.2)
gravity.system = system  
system.Add(gravity)


emitter = ParticleEmitter(system, gravity)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, 10))
vis.AddTypicalLights()


time_step = 0.01
current_time = 0

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    
    
    emitter.emit(current_time)
    
    
    system.DoStepDynamics(time_step)
    current_time += time_step