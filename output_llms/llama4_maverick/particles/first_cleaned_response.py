import pychrono as chrono
import pychrono.irrlicht as chronoirr
import random


chrono.SetChronoDataPath("/path/to/chrono/data/")  


sys = chrono.ChSystemSMC()
sys.Set_G_acc(chrono.ChVectorD(0, 0, 0))  


particle_container = chrono.ChBody()
particle_container.SetBodyFixed(True)
sys.Add(particle_container)


class ParticleEmitter:
    def __init__(self, system):
        self.system = system

    def EmitParticle(self):
        
        if random.random() < 0.5:
            shape = chrono.ChSphereShape()
            shape.GetSphereGeometry().rad = random.uniform(0.1, 0.5)
        else:
            shape = chrono.ChBoxShape()
            shape.GetBoxGeometry().Size = chrono.ChVectorD(random.uniform(0.1, 0.5), random.uniform(0.1, 0.5), random.uniform(0.1, 0.5))

        body = chrono.ChBody()
        body.SetMass(1)
        body.SetInertiaXX(chrono.ChVectorD(1, 1, 1))
        body.SetPos(chrono.ChVectorD(random.uniform(-5, 5), random.uniform(-5, 5), random.uniform(-5, 5)))
        body.SetPos_dt(chrono.ChVectorD(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1)))
        body.SetRot(chrono.Q_from_Euler123(chrono.ChVectorD(random.uniform(-chrono.CH_C_PI, chrono.CH_C_PI), random.uniform(-chrono.CH_C_PI, chrono.CH_C_PI), random.uniform(-chrono.CH_C_PI, chrono.CH_C_PI))))
        body.AddAsset(shape)

        self.system.Add(body)

        return body

emitter = ParticleEmitter(sys)


for _ in range(10):
    emitter.EmitParticle()


def ApplyCustomGravity(sys):
    bodies = sys.Get_bodylist()
    for i in range(len(bodies)):
        for j in range(i+1, len(bodies)):
            body_i = bodies[i]
            body_j = bodies[j]
            dist_vec = body_j.GetPos() - body_i.GetPos()
            dist = dist_vec.Length()
            if dist > 0:
                force = 0.1 * body_i.GetMass() * body_j.GetMass() / (dist * dist)
                force_vec = dist_vec * (force / dist)
                body_i.AddForce(force_vec)
                body_j.AddForce(-force_vec)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Particle Gravity Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0, 0, -10))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()
    
    
    if random.random() < 0.05:
        emitter.EmitParticle()
    
    
    ApplyCustomGravity(sys)
    
    
    sys.DoStepDynamics(0.01)
    
    vis.EndScene()