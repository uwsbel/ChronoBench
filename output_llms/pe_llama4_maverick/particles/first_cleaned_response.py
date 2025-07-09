import pychrono as chrono
import pychrono.irrlicht as chronoirr
import numpy as np


chrono.SetChronoDataPath('/path/to/chrono/data')


sys = chrono.ChSystemSMC()


class ParticleEmitter:
    def __init__(self, sys):
        self.sys = sys
        self.material = chrono.ChContactMaterialSMC()
        self.material.SetFriction(0.4)

    def EmitParticle(self):
        
        shape_type = np.random.choice(['sphere', 'box', 'cylinder'])
        
        if shape_type == 'sphere':
            radius = np.random.uniform(0.1, 0.3)
            body = chrono.ChBodyEasySphere(radius, 1000, True, True, self.material)
        elif shape_type == 'box':
            size = chrono.ChVector3d(np.random.uniform(0.1, 0.3), np.random.uniform(0.1, 0.3), np.random.uniform(0.1, 0.3))
            body = chrono.ChBodyEasyBox(size.x, size.y, size.z, 1000, True, True, self.material)
        else:  
            radius = np.random.uniform(0.1, 0.3)
            height = np.random.uniform(0.2, 0.5)
            body = chrono.ChBodyEasyCylinder(radius, height, 1000, True, True, self.material)

        
        body.SetPos(chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(1, 2), np.random.uniform(-1, 1)))
        body.SetPos_dt(chrono.ChVector3d(np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5), np.random.uniform(-0.5, 0.5)))
        body.SetRot(chrono.Q_from_AngAxis(np.random.uniform(-chrono.CH_PI, chrono.CH_PI), chrono.ChVector3d(np.random.uniform(-1, 1), np.random.uniform(-1, 1), np.random.uniform(-1, 1)).GetNormalized()))

        self.sys.Add(body)


emitter = ParticleEmitter(sys)


vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle('Gravitational Attraction Simulation')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 3, 6))
vis.AddTypicalLights()


while vis.Run():
    vis.BeginScene()
    vis.Render()

    
    if sys.GetChTime() % 0.1 < 0.01:
        emitter.EmitParticle()

    
    for body in sys.Get_bodylist():
        if body.GetPos().y > 0:
            force = chrono.ChVector3d(0, -9.81, 0) * body.GetMass()
            body.Empty_forcelist()
            body.Accumulate_force(force, body.GetPos(), False)

        
        for other_body in sys.Get_bodylist():
            if body != other_body:
                distance = (body.GetPos() - other_body.GetPos()).Length()
                if distance > 0:
                    force_dir = (other_body.GetPos() - body.GetPos()).GetNormalized()
                    force_mag = 0.1 * body.GetMass() * other_body.GetMass() / (distance * distance)
                    force = force_dir * force_mag
                    body.Accumulate_force(force, body.GetPos(), False)

    sys.DoStepDynamics(0.01)
    vis.EndScene()