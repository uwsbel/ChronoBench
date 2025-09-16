import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.pardisomkl as chronopardisomkl
import numpy as np


sys = chrono.ChSystemNSC()


contact_material = chrono.ChMaterialSurfaceNSC()
contact_material.SetFriction(0.5)
contact_material.SetRestitution(0.5)
contact_material.SetYoungModulus(1e6)


class ParticleEmitter:
    def __init__(self, sys):
        self.sys = sys
        self.particles = []

    def emit_particle(self):
        
        shape = np.random.choice(['sphere', 'box', 'cylinder'])
        pos = chrono.ChVectorD(np.random.uniform(-10, 10), np.random.uniform(-10, 10), np.random.uniform(-10, 10))
        vel = chrono.ChVectorD(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1))
        rot = chrono.Q_from_AngXAxis(np.random.uniform(0, np.pi))

        
        if shape == 'sphere':
            particle = chrono.ChBodyEasySphere(self.sys, 1, 1000, contact_material)
        elif shape == 'box':
            particle = chrono.ChBodyEasyBox(self.sys, 1, 1, 1, 1000, contact_material)
        elif shape == 'cylinder':
            particle = chrono.ChBodyEasyCylinder(self.sys, 1, 1, 1000, contact_material)

        
        particle.SetPos(pos)
        particle.SetPos_dt(vel)
        particle.SetRot(rot)

        
        self.sys.Add(particle)
        self.particles.append(particle)


emitter = ParticleEmitter(sys)
for _ in range(100):
    emitter.emit_particle()


class CustomGravitationalForce(chrono.ChForce):
    def __init__(self):
        super().__init__()

    def ComputeF(self, x, v, m):
        force = chrono.ChVectorD(0, 0, 0)
        for particle in emitter.particles:
            if particle != self.GetBody():
                distance = (particle.GetPos() - self.GetBody().GetPos()).Length()
                force += (particle.GetPos() - self.GetBody().GetPos()) / distance * 0.1
        return force


for particle in emitter.particles:
    force = CustomGravitationalForce()
    particle.AddForce(force)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVectorD(0, 0, 20))
vis.AddTypicalCameraTracks()
vis.SetWindowSize(1024, 768)
vis.SetLogo("")


while vis.Run():
    sys.DoStepDynamics(0.01)
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()