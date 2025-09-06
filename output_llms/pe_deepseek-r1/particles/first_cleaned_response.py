import os
import math
import random
import pychrono as chrono
import pychrono.irrlicht as chronoirr


chrono.SetChronoDataPath(os.path.join(os.path.dirname(__file__), 'data/'))


sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravity(chrono.ChVector3d(0, 0, 0))  


material = chrono.ChContactMaterialSMC()
material.SetFriction(0.2)
material.SetRestitution(0.1)
material.SetAdhesion(0)  


class GravitationalParticle(chrono.ChBody):
    def __init__(self):
        super().__init__()
        self.mass = random.uniform(0.5, 2.0)
        self.SetMass(self.mass)
        self.SetInertiaXX(chrono.ChVector3d(1, 1, 1))
        self.SetCollide(True)
        
    def AddGravityForce(self, other, G=1.0):
        r = other.GetPos() - self.GetPos()
        dist = r.Length()
        if dist > 0:
            force_mag = G * self.mass * other.mass / (dist * dist)
            force_dir = r.GetNormalized()
            self.AccumulateForce(force_dir * force_mag, self.GetPos(), False)


class ParticleEmitter:
    def __init__(self, system):
        self.system = system
        self.particles = []
        self.last_emission_time = 0
        self.emission_interval = 0.1  
        
    def emit(self, current_time):
        if current_time - self.last_emission_time >= self.emission_interval:
            
            particle = GravitationalParticle()
            
            
            pos = chrono.ChVector3d(
                random.uniform(-1.5, 1.5),
                random.uniform(3.0, 6.0),
                random.uniform(-1.5, 1.5)
            )
            particle.SetPos(pos)
            
            
            vel = chrono.ChVector3d(
                random.uniform(-0.5, 0.5),
                random.uniform(-1.0, 0),
                random.uniform(-0.5, 0.5)
            )
            particle.SetPos_dt(vel)
            
            
            rot = chrono.ChQuaterniond()
            rot.SetFromAngleAxis(random.uniform(0, 2*math.pi), 
                                chrono.ChVector3d(random.random(), 
                                                 random.random(), 
                                                 random.random()).GetNormalized())
            particle.SetRot(rot)
            
            
            shape_type = random.choice(['sphere', 'box', 'cylinder'])
            color = chrono.ChColor(random.random(), random.random(), random.random())
            
            if shape_type == 'sphere':
                radius = random.uniform(0.1, 0.3)
                shape = chrono.ChCollisionShapeSphere(material, radius)
                particle.AddCollisionShape(shape)
                vis_shape = chrono.ChVisualShapeSphere(radius)
                vis_shape.SetColor(color)
                particle.AddVisualShape(vis_shape)
                
            elif shape_type == 'box':
                size = chrono.ChVector3d(
                    random.uniform(0.1, 0.4),
                    random.uniform(0.1, 0.4),
                    random.uniform(0.1, 0.4)
                )
                shape = chrono.ChCollisionShapeBox(material, size.x, size.y, size.z)
                particle.AddCollisionShape(shape)
                vis_shape = chrono.ChVisualShapeBox(size.x, size.y, size.z)
                vis_shape.SetColor(color)
                particle.AddVisualShape(vis_shape)
                
            elif shape_type == 'cylinder':
                radius = random.uniform(0.1, 0.25)
                height = random.uniform(0.2, 0.5)
                shape = chrono.ChCollisionShapeCylinder(material, radius, height)
                particle.AddCollisionShape(shape)
                vis_shape = chrono.ChVisualShapeCylinder(radius, height)
                vis_shape.SetColor(color)
                particle.AddVisualShape(vis_shape)
            
            
            self.system.Add(particle)
            self.particles.append(particle)
            self.last_emission_time = current_time
            
    def apply_mutual_gravity(self, G=1.0):
        for i, p1 in enumerate(self.particles):
            for p2 in self.particles[i+1:]:
                p1.AddGravityForce(p2, G)
                p2.AddGravityForce(p1, G)


emitter = ParticleEmitter(sys)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280, 720)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(0, 3, -8))
vis.AddTypicalLights()
vis.AddSkyBox()


time_step = 0.005
current_time = 0
max_particles = 30

while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.RenderGrid(chrono.ChCoordsysd(), 2, 2, 12, 12, 
                   chrono.ChColor(0.2, 0.2, 0.2), True)
    vis.EndScene()
    
    
    if len(emitter.particles) < max_particles:
        emitter.emit(current_time)
    
    
    emitter.apply_mutual_gravity(G=0.5)  
    
    
    sys.DoStepDynamics(time_step)
    current_time += time_step