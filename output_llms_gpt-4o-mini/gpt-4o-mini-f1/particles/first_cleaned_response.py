import pychrono as ch
import pychrono.irrlicht as irr
import math
import random


sys = ch.ChSystemNSC()
sys.SetCollisionSystemType(ch.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, 0))  


class ParticleEmitter:
    def __init__(self, system, num_particles):
        self.system = system
        self.num_particles = num_particles

    def emit_particles(self):
        for _ in range(self.num_particles):
            
            particle = ch.ChBodyEasySphere(0.5, 1000, True, True)
            particle.SetPos(ch.ChVector3d(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10)))
            particle.SetRot(ch.ChQuaterniond(1, 0, 0, 0))
            particle.SetLinVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle.SetAngVel(ch.ChVector3d(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
            particle.GetMaterialSurfaceNSC().SetFriction(0.1)
            particle.GetMaterialSurfaceNSC().SetRestitution(0.1)
            particle.SetFixed(False)
            particle.SetCollide(True)
            self.system.Add(particle)


emitter = ParticleEmitter(sys, 10)


vis = irr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Emitter with Custom Gravity')
vis.Initialize()
vis.AddLogo(ch.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(ch.ChVector3d(0, 0, 10))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    
    if sys.GetChTime() < 0.1:
        emitter.emit_particles()

    
    for body in sys.Get_bodylist():
        if body.IsFixed():
            continue
        force = ch.ChVector3d(0, 0, 0)
        for other in sys.Get_bodylist():
            if other is body or other.IsFixed():
                continue
            r = body.GetPos() - other.GetPos()
            distance = r.Length()
            if distance < 1e-3:
                continue  
            force += (r.Normalize() * (body.GetMass() * other.GetMass()) / (distance * distance))
        body.AddForce(force)

    
    sys.DoStepDynamics(1e-3)